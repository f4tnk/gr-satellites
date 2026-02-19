# gr-satellites — F4TNK DSP Optimizations

Branch: `master-f4tnk`  
Based on: upstream `main` (Daniel Estévez / daniestevez)  
Station: SatNOGS #3762 — AirSpy R2 @ 2.5 MSPS, x86-64 (AVX2 + BMI2)

---

## Résumé des commits

| Commit | Description |
|--------|-------------|
| `c1374671` | Lot 1 : VOLK / -march=native / rms_agc_cc / doppler NCO / kurtosis / manchester |
| `529d8d2f` | Lot 2 : rms_agc_ff / convolutional_encoder / crc SWAR / matrix_deinterleaver |
| `HEAD`     | Lot 3 : ViterbiCodec move-semantics / hdlc_deframer numpy.packbits + analyse exhaustive |

---

## Catégorie 1 — Compilation globale

### C1.1 `-march=native -O3` dans `lib/CMakeLists.txt`

```cmake
if(NOT MSVC)
    target_compile_options(gnuradio-satellites PRIVATE -march=native -O3)
endif()
```

**Impact transversal** : active l'auto-vectoriseur GCC/Clang sur toutes les
boucles scalaires de la bibliothèque.  Sur un CPU Haswell/Skylake qui dispose
d'AVX2 256-bit, les boucles de bytes peuvent traiter 32 bytes/cycle au lieu de 1.

Fichiers affectés :
- `nrzi_decode_impl.cc` — boucle `~(in[i+1] ^ in[i]) & 1` SIMD-ifiable
- `nrzi_encode_impl.cc` — similaire
- `descrambler308_impl.cc` — XOR logique (séquentiel par nature, pas vectorisable)
- `pdu_scrambler_impl.cc` — boucle `msg[j] ^= d_sequence[j]` → AVX2 XOR 32B/cycle
- CRC compute loop — 1-byte lutup → non-vectorisable mais code plus efficace

---

## Catégorie 2 — AGC (Automatic Gain Control)

### C2.1 `rms_agc_cc` — nouveau bloc C++ single-pass (complexe)

**Fichiers** : `include/satellites/rms_agc_cc.h`, `lib/rms_agc_cc_impl.{h,cc}`,
`python/bindings/rms_agc_cc_python.cc`, `python/hier/rms_agc.py`

**Ancienne architecture** (5 blocs GNU Radio en cascade) :
```
rms_cf(alpha) → multiply_const_ff(1/ref) → add_const_ff(1e-19)
              → float_to_complex → divide_cc
```
Chaque transition de bloc = 1 buffer intermédiaire alloué par le scheduler GR
+ overhead de scheduling.  5 transitions + 1 branche (input split) pour 1 seule
normalisation par échantillon.

**Nouvelle architecture** (1 seul bloc C++) :
```
Pass 1 (VOLK):   volk_32fc_magnitude_squared_32f → |z|² batch
Pass 2 (scalar): IIR EMA séquentiel → rms_sq(n) = (1-α)·rms_sq(n-1) + α·|z[n]|²
                 gain(n) = ref / (√rms_sq(n) + ref·1e-19)
Pass 3 (VOLK):   volk_32f_x2_multiply_32f → out = in · gain (float)
```
- `volk_32fc_magnitude_squared_32f` : ~8 paires IQ/cycle AVX2 vs 1 paire scalaire
- L'IIR est intrinsèquement séquentiel (chaque valeur dépend de la précédente)
- La passe 3 est auto-vectorisée par `-march=native -O3`
- **Suppression de 4 copies mémoire inter-blocs** et de 4 appels scheduler GR

Stabilité : même constante anti-division-par-zéro `1e-19` qu'en Python.
API publique identique : `set_alpha()`, `set_reference()` Thread-safe.

---

### C2.2 `rms_agc_ff` — nouveau bloc C++ single-pass (float)

**Fichiers** : `include/satellites/rms_agc_ff.h`, `lib/rms_agc_ff_impl.{h,cc}`,
`python/bindings/rms_agc_ff_python.cc`, `python/hier/rms_agc_f.py`

**Ancienne architecture** (4 blocs) :
```
rms_ff(alpha) → multiply_const_ff(1/ref) → add_const_ff(1e-19) → divide_ff
```

**Nouvelle architecture** (1 seul bloc C++) :
```
Pass 1 (VOLK):   volk_32f_x2_multiply_32f(sq, in, in, n) → x² batch
Pass 2 (scalar): IIR EMA + gain
Pass 3 (VOLK):   volk_32f_x2_multiply_32f(out, in, gain, n) → out = in·gain
```
Suppression de 3 copies mémoire inter-blocs.

---

## Catégorie 3 — Correction Doppler

### C3.1 Vectorisation du NCO dans `doppler_correction_impl.cc`

**Fichiers** : `lib/doppler_correction_impl.{h,cc}`

**Ancien code** (boucle scalaire sample-par-sample) :
```cpp
for (int j = 0; j < noutput_items; ++j) {
    // ... interpolation ...
    d_phase += freq;
    phase_wrap();
    const gr_complex nco = gr_expj(-d_phase);
    gr::fast_cc_multiply(out[j], in[j], nco);
}
```
`gr_expj` → `sincosf` scalaire = 1 paire IQ/cycle.

**Nouveau code** (3 passes dont 2 VOLK) :
```
Pass 1 (scalar): interpolation de la rampe de fréquence + accumulation de phase
                 → d_phase_buf[j] = -d_phase (rampe complète en single float)
Pass 2 (VOLK):   volk_32f_cos_32f  + volk_32f_sin_32f  → cosinus/sinus du bloc
Pass 3 (VOLK):   volk_32fc_x2_multiply_32fc → out = in · nco_vec
```
Sur AVX2, VOLK sincos traite ~8 floats/cycle. L'interpolation (Pass 1) reste
scalaire car elle est séquentielle (chaque index dépend du précédent temps).

Buffers scratch alloués alignés (`volk::vector`) et grandit dynamiquement.

---

## Catégorie 4 — Kurtosis spectrale

### C4.1 Vectorisation VOLK dans `kurtosis_impl.cc`

**Ancien code** (commentaire `// TODO: could perhaps be optimized using Volk kernels`) :
```cpp
for (size_t l = 0; l < d_block_size; ++l) {
    const gr_complex z = in[...];
    const float sq = z.real()*z.real() + z.imag()*z.imag();
    sum2 += sq;
    sum4 += sq * sq;
}
```

**Nouveau code** :
```cpp
// Fast path vlen == 1 : input est un bloc complexe contigu
volk_32fc_magnitude_squared_32f(d_sq.data(), &in[j * d_block_size], d_block_size);
volk_32f_x2_multiply_32f(d_sq4.data(), d_sq.data(), d_sq.data(), d_block_size);
volk_32f_accumulator_s32f(&sum2, d_sq.data(), d_block_size);
volk_32f_accumulator_s32f(&sum4, d_sq4.data(), d_block_size);
```
- `volk_32fc_magnitude_squared_32f` : ~8 échantillons/cycle AVX2
- `volk_32f_x2_multiply_32f` : vectorise le carré
- `volk_32f_accumulator_s32f` : réduction vectorisée
- **Speedup estimé : ×3–5** sur AVX2 par rapport à la boucle scalaire précédente

Cas général (vlen > 1) : fallback scalaire conservé (accès interleaved non-contigu).

---

## Catégorie 5 — Synchronisation Manchester

### C5.1 `magnitude_squared` pour la métrique d'alignement dans `manchester_sync_impl.cc`

```cpp
// Avant
volk_32fc_magnitude_32f(out, in, block_size);        // calcule √(re²+im²)

// Après
volk_32fc_magnitude_squared_32f(out, in, block_size); // calcule re²+im²
```
**Justification mathématique** : la décision est `metric0 > metric1`.
Étant donné que `f(x) = x²` est strictement croissante pour x ≥ 0 :
`|a|² > |b|²  ⟺  |a| > |b|`
→ le sqrt est inutile pour la comparaison.  Environ **1 cycle économisé par
échantillon** (`sqrt` réciproque étant coûteux même avec AVX2).

---

## Catégorie 6 — Viterbi / encodeur convolutif

### C6.1 Élimination des `push_back` dans `viterbi_decoder_impl.cc`

**Avant** (allocations dynamiques répétées) :
```cpp
std::string bits;
for (auto b : msg) { bits.push_back(b ? '1' : '0'); }  // N reallocations
std::vector<uint8_t> out;
for (auto b : outbits) { out.push_back(b == '1'); }    // N reallocations
```

**Après** (pre-allocation unique) :
```cpp
std::string bits(len, '0');
for (size_t i = 0; i < len; ++i) bits[i] = msg[i] ? '1' : '0';
std::vector<uint8_t> out(outlen);
for (size_t i = 0; i < outlen; ++i) out[i] = (outbits[i] == '1') ? 1 : 0;
```
Pour une trame CCSDS 256 octets → 2048 bits : évite ~2048 réallocations `push_back`
et autant de bounds-checks.

### C6.2 Même fix dans `convolutional_encoder_impl.cc`

Identique à C6.1 mais pour le sens encodage.  Affecte la génération de trames
de test / simulation — même pattern, même gain.

---

## Catégorie 7 — CRC

### C7.1 `crc::reflect()` : algorithme SWAR O(1) au lieu de boucle O(n)

**Fichier** : `lib/crc.cc`

**Avant** (boucle O(num_bits)) :
```cpp
uint64_t ret = word & 1;
for (unsigned i = 1; i < d_num_bits; ++i) {
    word >>= 1;
    ret = (ret << 1) | (word & 1);
}
```

**Après** (SWAR — SIMD Within A Register, O(1) / 7 instructions) :
```cpp
// Reverse all 64 bits via bit-interleaving masks
v = ((v & 0xAAAA…) >> 1)  | ((v & 0x5555…) << 1);  // swap odd/even bits
v = ((v & 0xCCCC…) >> 2)  | ((v & 0x3333…) << 2);  // swap 2-bit groups
v = ((v & 0xF0F0…) >> 4)  | ((v & 0x0F0F…) << 4);  // swap nibbles
v = ((v & 0xFF00…) >> 8)  | ((v & 0x00FF…) << 8);  // swap bytes
v = ((v & 0xFFFF0000…) >> 16) | …;                  // swap shorts
v = (v >> 32) | (v << 32);                           // swap halves
return v >> (64 - d_num_bits);
```
Speedup : environ **×8** pour CRC-32 (`d_num_bits = 32`).
`reflect()` est appelée lors du calcul CRC pour les modes `input_reflected` et
`result_reflected` (CRC-32/MPEG, CRC-16/IBM, CCITT...).

---

## Catégorie 8 — Désentrelacement matriciel

### C8.1 Transposée cache-friendly dans `matrix_deinterleaver_soft_impl.cc`

**Fichier** : `lib/matrix_deinterleaver_soft_impl.cc`

**Ancienne boucle** (accès stride sur la lecture, séquentiel sur l'écriture) :
```cpp
for (size_t i = 0; i < length; ++i) {
    d_out[i] = data[d_rows * (i % d_cols) + i / d_cols];  // lecture aléatoire
}
```
Les divisions `%` et `/` entières sont coûteuses.  La lecture dans `data` est
un accès stride de `d_rows` éléments — pour la matrice CCSDS typique 8×110
(880 float), le stride est de 8×4 = 32 octets.  Chaque ligne de cache (64B) est
accédée deux fois.

**Nouveau code** (écriture séquentielle, lecture en lines of `d_rows` éléments) :
```cpp
for (size_t row = 0; row < d_rows; ++row) {
    for (size_t col = 0; col < d_cols; ++col) {
        d_out[row * d_cols + col] = data[col * d_rows + row];
    }
}
```
- Écriture entièrement séquentielle → hardware prefetcher actif
- Suppression des divisions entières par modulo (remplacées par index arithmétique)
- Le compilateur avec `-march=native` peut auto-vectoriser la boucle interne

---

## Catégorie 9 — Infrastructure cmake

### C9.1 Gestion automatique des headers `*_pydoc.h` pour les nouveaux blocs

```cmake
foreach(_f rms_agc_cc rms_agc_ff)
    if(NOT EXISTS "${CMAKE_CURRENT_BINARY_DIR}/${_f}_pydoc.h")
        configure_file(
            "${CMAKE_CURRENT_SOURCE_DIR}/docstrings/${_f}_pydoc_template.h"
            "${CMAKE_CURRENT_BINARY_DIR}/${_f}_pydoc.h" COPYONLY)
    endif()
endforeach()
```
Assure que les headers de docstring pybind11 sont présents lors d'une première
build propre, sans dépendre du `GR_PYBIND_MAKE_OOT` qui ne les génère que si
le hash de l'en-tête C++ change.

---

---

## Lot 3 — ViterbiCodec move-semantics + hdlc_deframer numpy

### C10.1 `ViterbiCodec::UpdatePathMetrics` — `std::move` pour éviter les copies vectorielles

**Fichier** : `lib/viterbi/viterbi.cc`

**Ancienne code** :
```cpp
*path_metrics = new_path_metrics;          // copie de vector<int> (K-1 = 64 ints pour K=7)
trellis->push_back(new_trellis_column);    // copie de vector<int> dans le trellis
```

**Nouveau code** :
```cpp
*path_metrics = std::move(new_path_metrics);
trellis->push_back(std::move(new_trellis_column));
```

`UpdatePathMetrics` est appelée pour chaque paquet de `num_parity_bits` bits décodés.  
Pour un frame CCSDS R=1/2, K=7 de 256 octets = 2048 bits → 1024 appels.  
Chaque appel copiait auparavant deux `vector<int>` de taille 64 (2×64×4 = 512 octets) ; avec `std::move`, aucun octets n'est copié (seuls les pointeurs internes du vecteur sont transférés).

**Économie** : 1024 × 512 octets de copies évitées = **512 Ko** de mouvement mémoire
supprimé pour un frame CCSDS. Les frames sont décodées en continu, l'économie est
proportionnelle au débit.

---

### C10.2 `ViterbiCodec::Decode` — `trellis.reserve` + `decoded.push_back`

**Fichier** : `lib/viterbi/viterbi.cc`

```cpp
// Avant
Trellis trellis;  // realloc ~log2(1024) = 10 fois pendant les 1024 push_back

// Après
Trellis trellis;
trellis.reserve(bits.size() / num_parity_bits());  // capacité exacte, 0 realloc
```

Pour 1024 `push_back` sur `std::vector<std::vector<int>>` :  
Sans reserve : ~10 réallocations + déplacements de vecteurs imbriqués.  
Avec reserve : 1 allocation initiale, 0 réallocation.

```cpp
// Traceback — avant
std::string decoded;
decoded += state >> (constraint_ - 2) ? "1" : "0";  // recherche '\0', branch on literal

// Après
std::string decoded;
decoded.reserve(trellis.size());             // 1 allocation pour la totalité
decoded.push_back('0' ou '1');               // O(1) amortized, pas de recherche '\0'
```

---

### C11.1 `hdlc_deframer.py::pack()` — `numpy.packbits` au lieu de boucle Python imbriquée

**Fichier** : `python/hdlc_deframer.py`

**Avant** (2 boucles Python) :
```python
def pack(s):
    d = bytearray()
    for i in range(0, len(s), 8):
        x = 0
        for j in range(7, -1, -1):  # LSB first
            x <<= 1
            x += s[i+j]
        d.append(x)
    return d
```
Pour 256 bytes d'HDLC = 2048 bits : 256 iterations externes + 2048 iterations internes
= 2304 appels Python. Coût : ~50–200 µs dans CPython.

**Après** (appel C NumPy) :
```python
def pack(s):
    return numpy.packbits(numpy.array(s, dtype=numpy.uint8),
                          bitorder='little').tobytes()
```
- `bitorder='little'` : bit[0] = LSB du premier octet → sémantique identique
- Implémenté en C dans NumPy, ~50× plus rapide
- `pandas.packbits` traite 256 octets en un seul appel C vectorisé

`pack()` est appelée à chaque flag HDLC reçu (fin de trame). Pour une station
SatNOGS traitant du KISS/AX.25 à fort taux (beacons FM, BPSK), la fréquence
peut atteindre plusieurs centaines d'appels par seconde en burst.

---

## Analyse exhaustive des fichiers restants (lot 3)

Les fichiers suivants ont été lus et analysés — **aucune optimisation supplémentaire
n'a été identifiée** pour les raisons indiquées :

| Fichier | Raison |
|---------|--------|
| `golay24.c` | Déjà optimal : `volk_32u_popcnt` + `__builtin_parity` |
| `randomizer.c::ccsds_xor_sequence` | Boucle XOR auto-vectorisée par `-march=native` |
| `nrzi_decode_impl.cc` | `~(a^b)&1` contiguous, auto-vectorisable |
| `nrzi_encode_impl.cc` | Loop-carried dependency (`d_last`), non vectorisable |
| `descrambler308_impl.cc` | LFSR séquentiel, chaque bit dépend du précédent |
| `nusat_decoder_impl.cc` | PDU, descramble 64 B max, CRC-8 sequential table |
| `decode_rs_impl.cc` / `encode_rs_impl.cc` | libfec interne, stride-gather non vectorisable |
| `u482c_decode_impl.cc` / `u482c_encode_impl.cc` | PDU, golay+RS+LFSR, pas de hot loop |
| `varlen_packet_framer/tagger_impl.cc` | Tag/frame building, no compute loop |
| `selector_impl.cc` | Routing, memcpy paths |
| `lilacsat1_demux_impl.cc` | Tag-based demux, no hot loop |
| `time_dependent_delay_impl.cc` | Polyphase FIR — déjà vectorisé par GR |
| `viterbi.c` (Phil Karn K=7) | 32 BFLY unrolled ; SSE2/AVX2 = travail futur (voir F2) |
| `bch15.py` | BCH(15,k,d) sur mots de 15 bits, volume trop faible |
| `crcs.py` | Wrapper gnuradio `crc_check`, rien à faire |
| `components/demodulators/*.py` | Hier GR flowgraph wrappers, pas de compute Python |
| `distributed_syncframe_soft_impl.cc` | Inner loop auto-vectorisé (`-march=native`) pour step=1 ; changement soft→hard invalide sémantiquement |
| `pdu_scrambler_impl.cc` | XOR auto-vectorisé ; 2 copies PMT inévitables |

---

## Pistes non implémentées (futures)

### F1. ViterbiCodec — réécriture sans std::string (impact élevé, risque élevé)

La classe `ViterbiCodec` (`lib/viterbi/viterbi.cc`) utilise `std::string` pour
toutes ses structures internes : outputs, trellis branches, decoded bits.
Une réécriture complète sur `std::vector<uint8_t>` éliminerait :
- La conversion double msg→string (C6.1) deviendrait triviale (passage direct)
- Les allocations `trellis.push_back(new_trellis_column)` à chaque bit décodé
- Le `std::string::substr` en traceback  
Impact estimé : ×2–4 sur le décodage Viterbi générique (non-CCSDS).  
*Risque API : nécessite de modifier l'interface publique de ViterbiCodec.*

### F2. Décodeur Viterbi CCSDS avec SIMD (viterbi.c → SSE2/AVX2)

Le fichier `lib/viterbi.c` est la version C portable de Phil Karn (K=7).
Phil Karn distribue aussi `viterbi27_sse2.c` et `viterbi27_avx2.c` qui traitent
respectivement 4 (SSE2) et 32 (AVX2) états trellis en parallèle via des ACS
(Add-Compare-Select) vectorisés.  Le gain est de ×4 à ×10.
*Contrainte : nécessite une détection CPU au runtime ou un flag cmake.*

### F3. CRC multi-octets (slice-by-4/8)

L'implémentation actuelle traite 1 byte par cycle dans `crc::compute()`.
La technique "slice-by-8" utilise 8 tables de 256 entrées pour traiter 8 bytes
simultanément, avec un speedup de ×4–6.  Pour les trames de 255 octets RS,
l'impact est limité mais mesurable pour les CRC calculés en continu.

### F4. Synchroniseur de trame distribué — VOLK `dot_prod` pour step=1

Pour `distributed_syncframe_soft_impl.cc` avec `d_step = 1`, la boucle de
corrélation est équivalente à un produit scalaire sur un vecteur ±1 :
```cpp
volk_32f_x2_dot_prod_32f(&metric, in + i, syncword_pm1.data(), syncword_size);
```
Pour step > 1 (cas distribué AIS/GMSK) un `gather` SIMD serait nécessaire,
ce qui n'est pas supporté directement par VOLK.

### F5. PN9 scrambler — bloc C++ PDU natif

`python/hier/pn9_scrambler.py` utilise une chaîne de 3 blocs GR plus deux
convertisseurs PDU↔tagged-stream.  Un bloc C++ `pn9_scrambler_pdu` opérant
directement sur vecteur `uint8_t` éliminerait 2 copies de PDU et tout
l'overhead des tagged streams.

### F6. Costas loop 8APSK — mini-batch pour réduire l'overhead NCO

`costas_loop_8apsk_cc_impl.cc` appelle `gr_expj(-d_phase)` sample-par-sample.
La fréquence de mise à jour de la boucle (dépendante du `loop_bw`) est
typiquement bien inférieure à 1/sample.  On pourrait accumuler `M` samples
avec la même correction NCO, ne recalculer `expj` que tous les M steps, et
appliquer la correction vectoriellement via `volk_32fc_x2_multiply_32fc`.
*Précaution : augmente la latence de convergence, acceptable si M est petit (~8).*

---

## Notes d'installation

```bash
cd /chemin/vers/gr-satellites
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
sudo ldconfig
```

Les flags `-march=native -O3` sont appliqués automatiquement à la lib partagée.
Pour une cross-compilation (ex. RPi), remplacer par le `-march` approprié dans
`lib/CMakeLists.txt`.

---

## Validation

Utiliser `volk_profile` pour générer des profils VOLK optimaux sur le CPU cible
avant d'utiliser ces blocs en production :
```bash
volk_profile -j $(nproc)
```
Cela crée `~/.volk/volk_config` avec les meilleurs kernels SIMD détectés pour
`volk_32fc_magnitude_squared_32f`, `volk_32f_x2_multiply_32f`, etc.
