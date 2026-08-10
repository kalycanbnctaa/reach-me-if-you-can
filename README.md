# Reach Me if You Can

Simulator dan solver **Inverse Kinematics (IK)** untuk lengan robot 2D dengan jumlah sendi $N$ yang dapat dikonfigurasi ($2 \le N \le 5$). Dibangun sebagai submisi Task Seleksi Lab IRK 2026 menggunakan **Rust** dan **macroquad** untuk visualisasi GUI real-time.

Lengan robot digerakkan menuju titik target yang diklik pengguna melalui simulasi step-by-step dengan Forward Kinematics dan Jacobian yang diimplementasikan sepenuhnya *from scratch* (tanpa library IK/robotics siap pakai).

### Author

| NIM | Nama |
|---|---|
| 13524071 | Kalyca Nathania Benedicta Manullang |

---

## Daftar Isi

- [Fitur](#fitur)
- [Struktur Proyek](#struktur-proyek)
- [Instalasi dan Menjalankan](#instalasi--menjalankan)
- [Kontrol](#kontrol)
- [Konsep dan Implementasi](#konsep--implementasi)
  - [Forward Kinematics](#forward-kinematics)
  - [Jacobian](#jacobian)
  - [Inverse Kinematics Solver](#inverse-kinematics-solver)
  - [Deteksi dan Penanganan Singularity](#deteksi--penanganan-singularity)
  - [Reachability dan Batas Sudut Sendi](#reachability-dan-batas-sudut-sendi)
  - [Obstacle Avoidance](#obstacle-avoidance)
- [Pengujian](#pengujian)
- [Batasan yang Dipatuhi](#batasan-yang-dipatuhi)
- [Kredit](#kredit)

---

## Fitur

### Spesifikasi Wajib

| Fitur | Status | Lokasi Implementasi |
|---|---|---|
| Forward Kinematics via matriks transformasi homogen, $N$ tidak di-hardcode | Terpenuhi | `src/kinematics/forward.rs`, `src/math/matrix3.rs`, `src/math/transform.rs` |
| Jacobian eksplisit (dikonstruksi manual) | Terpenuhi | `src/kinematics/jacobian.rs` |
| IK numerik *from scratch* (Jacobian Transpose dan Pseudo-Inverse) | Terpenuhi | `src/kinematics/transpose.rs`, `src/kinematics/pseudoinverse.rs`, `src/linear_solver/` |
| Deteksi singularity / near-singularity | Terpenuhi | `src/kinematics/singularity.rs` |
| Penanganan singularity tanpa NaN/divergensi | Terpenuhi | `src/kinematics/pseudoinverse.rs`, `src/kinematics/solver.rs` |
| Deteksi unreachable (jarak & batas sudut sendi) | Terpenuhi | `robot/arm.rs` (`is_within_reach`), `kinematics/solver.rs` (`is_blocked_by_limits`) |
| Constraint sudut per sendi (aktif/nonaktif, rentang kustom) | Terpenuhi | `robot/limits.rs`, `gui/panel.rs` |
| Input target via klik mouse | Terpenuhi | `src/app.rs` (`handle_input`) |
| Input jumlah sendi $N$ dan panjang tiap segmen | Terpenuhi | `src/gui/panel.rs` |
| Visualisasi GUI step-by-step | Terpenuhi | `src/gui/renderer.rs`, `src/simulation/mod.rs` |

### Spesifikasi Bonus

| Fitur | Status | Lokasi Implementasi |
|---|---|---|
| Obstacle Avoidance (potential field) | Terpenuhi | `src/collision/`, `src/kinematics/solver.rs` |
| Damped Least Squares / Levenberg–Marquardt | Terpenuhi | `src/kinematics/damped_ls.rs` |

### Fitur Tambahan

- **4 mode solver IK** yang bisa dibandingkan langsung dari dropdown: Jacobian Transpose, Pseudo-Inverse (dengan auto-damping saat mendekati singular), Pseudo-Inverse Undamped (untuk demonstrasi kegagalan tanpa damping), dan Damped Least Squares.
- **Overlay Jacobian real-time** yang menampilkan matriks $2 \times N$ secara langsung di layar.
- **Obstacle interaktif**: tambah obstacle lingkaran/kotak dengan klik, hapus dengan klik kanan.
- **Kontrol manual per sendi** via keyboard, independen dari solver IK.
- **Kecepatan animasi** yang dapat diatur (iterasi solver per frame).

---

## Struktur Proyek

```
reach-me-if-you-can/
├── Cargo.toml
├── src/
│   ├── main.rs                  # Entry point
│   ├── lib.rs                   # Deklarasi modul
│   ├── config.rs                # Konstanta window, warna, nilai default
│   ├── app.rs                   # State aplikasi, main loop, input handling
│   ├── math/
│   │   ├── vector2.rs           # Vektor 2D + operasi aljabar
│   │   ├── matrix3.rs           # Matriks 3x3 (transformasi homogen)
│   │   ├── transform.rs         # Wrapper Transform di atas Matrix3
│   │   └── utils.rs             # Fungsi utilitas (clamp, normalisasi sudut)
│   ├── robot/
│   │   ├── arm.rs               # Struct RobotArm (agregat state lengan)
│   │   ├── joint.rs             # Struct Joint
│   │   ├── segment.rs           # Struct Segment (panjang)
│   │   ├── limits.rs            # Batasan sudut sendi (JointLimit)
│   │   ├── pose.rs              # Posisi seluruh sendi hasil FK
│   │   └── state.rs             # State solver (iterasi, error, status)
│   ├── kinematics/
│   │   ├── forward.rs           # Forward Kinematics
│   │   ├── jacobian.rs          # Konstruksi matriks Jacobian
│   │   ├── inverse.rs           # Trait IkSolver + IkConfig
│   │   ├── transpose.rs         # Solver: Jacobian Transpose
│   │   ├── pseudoinverse.rs     # Solver: Pseudo-Inverse (damped dan undamped)
│   │   ├── damped_ls.rs         # Solver: Damped Least Squares (LM)
│   │   ├── singularity.rs       # Analisis manipulability dan singularity
│   │   └── solver.rs            # Loop step-by-step IK per frame
│   ├── linear_solver/
│   │   ├── gaussian.rs          # Eliminasi Gauss + back-substitution (from scratch)
│   │   ├── inverse.rs           # Inversi matriks via Gaussian
│   │   ├── determinant.rs       # Determinan via eliminasi Gauss
│   │   └── pseudo.rs            # Pseudo-inverse dan damped pseudo-inverse
│   ├── collision/
│   │   ├── obstacle.rs          # Struct Obstacle (Circle / AABB)
│   │   ├── distance.rs          # Jarak titik-ke-obstacle
│   │   ├── intersection.rs      # Deteksi tabrakan lengan-obstacle
│   │   └── potential_field.rs   # Gaya tolak untuk obstacle avoidance
│   ├── simulation/
│   │   └── mod.rs               # Orkestrasi simulasi (step per frame)
│   └── gui/
│       ├── panel.rs             # Panel kontrol kiri (N, panjang, solver, dll)
│       ├── renderer.rs          # Render lengan, target, obstacle, overlay
│       ├── widgets.rs           # Widget kustom (slider, checkbox, dropdown, dll)
│       ├── colors.rs            # Palet warna UI
│       ├── animation.rs         # Konstanta kecepatan animasi
│       └── input.rs             # Deteksi pointer di atas panel
└── tests/
    ├── fk_test.rs                # Uji Forward Kinematics
    ├── jacobian_test.rs          # Uji Jacobian analitik vs finite-difference
    └── dls_test.rs                # Uji stabilitas DLS di kondisi singular
```

---

## Instalasi dan Menjalankan

### Prasyarat

- [Rust](https://www.rust-lang.org/tools/install) (edisi 2024, disarankan versi stabil terbaru)
- Cargo (terpasang otomatis bersama Rust)

### Build dan Jalankan

```bash
git clone https://github.com/kalycanbnctaa/reach-me-if-you-can 
cd reach-me-if-you-can
cargo run --release
```

Mode `--release` sangat disarankan karena rendering dan iterasi solver berjalan jauh lebih responsif dibanding mode debug.

### Menjalankan Test

```bash
cargo test
```

Seluruh 11 test (`fk_test.rs`, `jacobian_test.rs`, `dls_test.rs`) harus lolos tanpa warning.

---

## Kontrol

### Mouse

| Aksi | Efek |
|---|---|
| Klik kiri (area kanvas) | Set titik target baru untuk end-effector |
| `O` + klik kiri | Tambah obstacle berbentuk lingkaran di posisi klik |
| `P` + klik kiri | Tambah obstacle berbentuk kotak (AABB) di posisi klik |
| Klik kanan | Hapus obstacle terdekat dari posisi klik |

### Keyboard

| Tombol | Efek |
|---|---|
| `R` | Reset lengan ke posisi awal (semua sudut sendi = 0) |
| `Space` | Pause / resume simulasi |
| `J` | Cetak matriks Jacobian saat ini ke console |
| `Q`/`A`, `W`/`S`, `E`/`D`, `Z`/`X`, `C`/`V` | Putar sendi 1–5 secara manual (searah/berlawanan jarum jam) |

### Panel Kontrol (kiri layar)

- **N (Joints)**: jumlah sendi (2–5), via tombol +/-
- **Segment Lengths**: slider panjang tiap segmen
- **Joint Limits**: checkbox untuk mengaktifkan batas sudut per sendi + slider rentang min/max
- **Display**: toggle tampilkan target dan overlay Jacobian
- **IK Solver**: dropdown pemilihan metode solver
- **Animation Speed**: jumlah iterasi solver per frame (mengatur kecepatan animasi)
- **Reset Arm (R)**: tombol reset

---

## Konsep dan Implementasi

### Forward Kinematics

Posisi setiap sendi dihitung dengan **chaining matriks transformasi homogen** $3 \times 3$ (rotasi + translasi digabung dalam satu matriks per sendi):

$$T_i = R(\theta_i) \cdot \text{Translate}(L_i, 0)$$

$$M_{\text{world}} = M_{\text{base}} \cdot T_1 \cdot T_2 \cdots T_N$$

Posisi end-effector adalah kolom translasi dari $M_{\text{world}}$ setelah seluruh sendi dirangkai. Implementasi berada di `Matrix3` (operasi matriks murni) dan `Transform` (wrapper semantik), lalu di-chain di `forward_kinematics()`.

Karena arah tiap segmen bergantung pada **sudut kumulatif** dari seluruh sendi sebelumnya, hasil akhir untuk sendi ke-$i$ (0-indexed) dapat dinyatakan sebagai:

$$\text{posisi akhir} = \text{base} + \sum_{i=0}^{N-1} L_i \cdot \big(\cos S_i,\ \sin S_i\big), \quad S_i = \theta_0 + \theta_1 + \cdots + \theta_i$$

$N$ dan panjang tiap segmen sepenuhnya parametrik (disimpan sebagai `Vec<f32>`/slice) sehingga tidak ada logika yang di-hardcode untuk nilai $N$ tertentu.

### Jacobian

Jacobian $J \in \mathbb{R}^{2 \times N}$ memetakan kecepatan sudut sendi ke kecepatan linear end-effector: $\dot{x} = J\dot{\theta}$. Setiap kolom dihitung secara analitik dari turunan parsial posisi end-effector terhadap $\theta_i$:

$$\frac{\partial x}{\partial \theta_i} = -\sum_{k=i}^{N-1} L_k \sin(S_k), \qquad \frac{\partial y}{\partial \theta_i} = \sum_{k=i}^{N-1} L_k \cos(S_k)$$

Formula ini konsisten dengan model Forward Kinematics di atas dan divalidasi di `tests/jacobian_test.rs` dengan membandingkannya terhadap turunan numerik (*central finite difference*).

### Inverse Kinematics Solver

Empat solver tersedia, semuanya diimplementasikan dari nol (`src/linear_solver/` hanya menyediakan operasi matriks dasar, perkalian, transpose, eliminasi Gauss, bukan fungsi *solve*/pseudo-inverse siap pakai dari library):

1. **Jacobian Transpose** (`transpose.rs`): $\Delta\theta = \alpha \cdot J^T e$. Sederhana dan selalu stabil, tapi konvergensi lebih lambat.
2. **Pseudo-Inverse** (`pseudoinverse.rs`): $\Delta\theta = J^T(JJ^T)^{-1} e$, dihitung via `right_pseudo_inverse()` yang dibangun di atas eliminasi Gauss buatan sendiri (`linear_solver/gaussian.rs`). Otomatis beralih ke versi *damped* saat mendeteksi kondisi (near-)singular.
3. **Pseudo-Inverse Undamped** (`pseudoinverse.rs`): versi murni tanpa auto-damping, disediakan khusus untuk **mendemonstrasikan** bagaimana solver konvensional bisa gagal (menghasilkan `None`/NaN) di kondisi singular sebagai pembanding terhadap DLS.
4. **Damped Least Squares / Levenberg–Marquardt** (`damped_ls.rs`): $\Delta\theta = J^T(JJ^T + \lambda^2 I)^{-1} e$ dengan $\lambda$ dihitung adaptif berdasarkan tingkat manipulability saat ini (mengikuti Nakamura & Hanafusa, 1986):

$$\lambda^2 = \left(1 - \left(\frac{\bar{\mu}}{\varepsilon}\right)^2\right)\lambda_{\max}^2 \quad \text{jika } \bar{\mu} < \varepsilon, \text{ else } 0$$

Semua solver mengimplementasikan trait `IkSolver` sehingga dapat dipertukarkan secara *runtime* lewat dropdown di panel GUI (`src/kinematics/inverse.rs`).

Simulasi berjalan secara **iteratif per frame** (bukan langsung snap ke solusi) melalui `simulation::update()` → `solver::step()`, sehingga pergerakan lengan menuju target terlihat sebagai animasi bertahap.

### Deteksi dan Penanganan Singularity

Kondisi singular dideteksi melalui **manipulability measure** Yoshikawa:

$$\mu = \sqrt{\det(JJ^T)}$$

dinormalisasi terhadap panjang total lengan agar konsisten di berbagai skala. Nilai ini dikategorikan menjadi tiga status di `singularity.rs`:

- **Normal**: $\mu_{\text{normalized}} > 0.05$
- **Near-singular**: $10^{-4} < \mu_{\text{normalized}} \le 0.05$
- **Singular**: $\mu_{\text{normalized}} \le 10^{-4}$ (atau tidak *finite*)

Penanganan dilakukan di dua lapis:

1. **Level solver**: `JacobianPseudoInverse` otomatis menambah faktor redaman ($\lambda$) saat status near-singular/singular terdeteksi sehingga sistem persamaan linear tidak pernah dibalik dalam kondisi *ill-conditioned* tanpa proteksi.
2. **Level orkestrasi**: `solver::step()` memeriksa setiap hasil $\Delta\theta$ dengan `is_finite()` sebelum diterapkan. Jika NaN terdeteksi, status ditandai `SINGULAR` dan iterasi dihentikan alih-alih mencemari state lengan dengan nilai tak-valid.

### Reachability dan Batas Sudut Sendi

Sesuai definisi pada spesifikasi tugas, sebuah target dianggap **tidak dapat dicapai** (*unreachable*) dalam dua kondisi berbeda dan program menangani keduanya secara terpisah karena sifatnya berbeda: yang pertama dapat diketahui *sebelum* solver berjalan, sedangkan yang kedua baru diketahui *setelah* solver mencoba.

**1. Unreachable secara jarak**

Dicek langsung sebelum iterasi solver dimulai, membandingkan jarak target ke base terhadap jangkauan minimum dan maksimum lengan:

$$L_{\min} \le \lVert \text{target} - \text{base} \rVert \le L_{\max}$$

dengan $L_{\max} = \sum L_i$ (total panjang segmen) dan $L_{\min} = \max(0,\ 2L_{\text{terpanjang}} - L_{\max})$, batas bawah ini muncul karena segmen terpanjang tidak bisa "dilipat penuh" oleh segmen-segmen lain jika sisanya lebih pendek darinya. Implementasi ada di `RobotArm::is_within_reach()` (`robot/arm.rs`). Jika target berada di luar rentang ini, status langsung ditandai `UNREACHABLE` tanpa membuang iterasi solver sama sekali.

**2. Unreachable karena batas sudut sendi**

Target bisa saja secara jarak berada dalam jangkauan, tapi tidak dapat dicapai karena `JointLimit` (`robot/limits.rs`) yang aktif pada satu atau lebih sendi membatasi konfigurasi yang bisa dicapai lengan. Berbeda dari kasus pertama, kondisi ini **tidak bisa dideteksi lebih dulu secara geometris sederhana**, solver perlu benar-benar mencoba mendekati target dan gagal karena mentok di batas sudut. Program mendeteksi ini di `solver::step()` melalui dua sinyal yang muncul bersamaan:

- Solver mencapai `max_iterations` tanpa konvergen (`error > position_tolerance`).
- Minimal satu sendi berada tepat di batas `min_angle`/`max_angle`-nya (`is_blocked_by_limits()`), menandakan solver "mentok" karena constraint, bukan karena kekurangan iterasi biasa.

Kombinasi ini ditandai dengan status `STALLED (LIMIT)`, dibedakan secara visual dari `STALLED` biasa (yang menandakan solver butuh lebih banyak iterasi tanpa sebab constraint) agar pengguna dapat langsung mengenali penyebab kegagalan konvergensi.

| Kondisi | Deteksi | Status |
|---|---|---|
| Jarak di luar $[L_{\min}, L_{\max}]$ | Sebelum solver berjalan | `UNREACHABLE` |
| Jarak valid, tapi batas sudut menghalangi | Setelah `max_iterations`, sendi mentok di limit | `STALLED (LIMIT)` |
| Jarak valid, sudut tidak terbatasi, tapi belum konvergen | Setelah `max_iterations` | `STALLED` |

### Obstacle Avoidance

Menggunakan pendekatan **artificial potential field**: setiap titik sampel di sepanjang segmen lengan (`collision::potential_field::compute_delta`) yang berada dalam radius pengaruh sebuah obstacle akan menghasilkan gaya tolak yang kemudian diproyeksikan ke ruang sudut sendi melalui Jacobian titik tersebut (bukan Jacobian end-effector) dan ditambahkan ke $\Delta\theta$ dari solver IK utama. Obstacle direpresentasikan sebagai lingkaran atau *axis-aligned bounding box* (AABB) dengan deteksi jarak bertanda (*signed distance*) yang juga digunakan untuk mendeteksi kolisi aktual (`collision::intersection`).

---

## Pengujian

| File | Cakupan |
|---|---|
| `tests/fk_test.rs` | Forward Kinematics: kasus sudut nol, rotasi 90°, jumlah sendi bervariasi, konsistensi `end_effector()` vs `RobotPose` |
| `tests/jacobian_test.rs` | Jacobian analitik dibandingkan terhadap *central finite difference*, dimensi matriks, kolom sendi terakhir |
| `tests/dls_test.rs` | Pseudo-inverse undamped gagal (mengembalikan `None`) tepat di titik singular, versi damped tetap *finite*, solver DLS menghasilkan delta valid baik di kondisi singular maupun normal |

Jalankan dengan:

```bash
cargo test
```

---

## Batasan yang Dipatuhi

Sesuai ketentuan spesifikasi tugas:

- Tidak menggunakan library IK/robotics siap pakai (IKPy, ROS, MoveIt, dsb).
- Tidak memanggil fungsi *solve*/pseudo-inverse/dekomposisi siap pakai dari library matriks manapun.
- `nalgebra` **hanya** digunakan sebagai *container* matriks generik (`DMatrix`) dan untuk operasi dasar (perkalian, transpose). Seluruh logika *solve* linear (eliminasi Gauss, pseudo-inverse, damped pseudo-inverse, determinan) diimplementasikan sendiri di `src/linear_solver/`.
- Solusi numerik berlaku generik untuk $N$ berapa pun dalam rentang $[2, 5]$ tidak ada percabangan logika berdasarkan nilai $N$ spesifik.

---

## Kredit

Dependensi eksternal:
- [`macroquad`](https://crates.io/crates/macroquad): framework rendering dan window management
- [`nalgebra`](https://crates.io/crates/nalgebra): struktur data matriks generik (`DMatrix`) sebagai container, bukan solver