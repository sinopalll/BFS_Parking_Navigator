import os
from collections import deque
from datetime import datetime

class Node:
    """
    Representasi satu kotak/sel di dalam lahan parkir.
    Menyimpan koordinat (x, y) dan referensi ke parent untuk backtracking.
    """
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent

class BFSParking:
    # ==========================================
    # KONSTANTA GLOBAL
    # ==========================================
    CELL_EMPTY = 0      # Jalan kosong yang dapat dilalui
    CELL_OBSTACLE = 1   # Tembok/dinding/rintangan
    CELL_SLOT = 2       # Slot parkir kosong
    CELL_GATE = 3       # Gerbang masuk
    CELL_PARKED = 4     # Slot parkir yang sudah terisi
    
    SIMBOL_MAP = {
        CELL_EMPTY: "░",      # Jalan
        CELL_OBSTACLE: "█",   # Tembok
        CELL_SLOT: "P",       # Parking slot kosong
        CELL_GATE: "S",       # Start/Gerbang
        CELL_PARKED: "#"      # Slot terisi
    }
    
    def __init__(self):
        self.peta = []
        self.peta_awal = []
        self.start_node = None
        self.goal_nodes = []
        self.riwayat_parkir = []  # Track mobil yang parkir
        self.ARAH = [
            (0, -1, "Utara"),
            (0,  1, "Selatan"),
            (-1, 0, "Barat"),
            (1,  0, "Timur"),
        ]

    # ==========================================
    # MODUL 1: MANAJEMEN LINGKUNGAN (FILE I/O)
    # ==========================================
    def muat_denah(self, nama_file):
        """
        Membaca matriks dari file teks dengan validasi format.
        
        Args:
            nama_file (str): Path file denah parkir
            
        Returns:
            bool: True jika berhasil, False jika ada error
        """
        if not os.path.exists(nama_file):
            print(f"[ERROR] File '{nama_file}' tidak ditemukan!")
            return False

        self.peta = []
        try:
            with open(nama_file, 'r') as file:
                for nomor_baris, baris in enumerate(file, 1):
                    if baris.strip() == "":
                        continue
                    try:
                        baris_data = [int(val) for val in baris.strip().split()]
                        # Validasi setiap cell hanya berisi nilai yang dikenali oleh konstanta CELL_
                        allowed_values = (
                            self.CELL_EMPTY,
                            self.CELL_OBSTACLE,
                            self.CELL_SLOT,
                            self.CELL_GATE,
                            self.CELL_PARKED,
                        )
                        for val in baris_data:
                            if val not in allowed_values:
                                print(f"[ERROR] Baris {nomor_baris}: nilai '{val}' tidak valid. Hanya gunakan nilai: {allowed_values}!")
                                return False
                        self.peta.append(baris_data)
                    except ValueError:
                        print(f"[ERROR] Baris {nomor_baris}: nilai bukan integer!")
                        return False

            # Validasi peta tidak kosong
            if not self.peta:
                print("[ERROR] File denah kosong!")
                return False
            
            # Validasi dimensi konsisten
            self.baris = len(self.peta)
            self.kolom = len(self.peta[0])
            for i, row in enumerate(self.peta, 1):
                if len(row) != self.kolom:
                    print(f"[ERROR] Baris {i}: jumlah kolom tidak konsisten! Diharapkan {self.kolom}, tapi dapat {len(row)}")
                    return False

            self.peta_awal = [row.copy() for row in self.peta]
            print(f"[SUKSES] Denah berhasil dimuat: {self.baris}x{self.kolom}")
            return True
            
        except Exception as e:
            print(f"[ERROR] Gagal membaca file: {str(e)}")
            return False

    def simpan_denah(self, nama_file):
        """
        Persistensi data: menimpa file txt dengan matriks terbaru.
        
        Args:
            nama_file (str): Path file untuk menyimpan denah
        """
        try:
            with open(nama_file, 'w') as file:
                for baris in self.peta:
                    baris_str = " ".join(str(sel) for sel in baris)
                    file.write(baris_str + "\n")
        except Exception as e:
            print(f"[ERROR] Gagal menyimpan file: {str(e)}")

    def cari_titik_penting(self):
        """
        Memindai ulang matriks untuk mencari Gerbang (3) dan Slot (2).
        Dipanggil setiap iterasi untuk sync dengan state terbaru.
        """
        self.goal_nodes = [] 
        self.start_node = None
        for y in range(self.baris):
            for x in range(self.kolom):
                if self.peta[y][x] == self.CELL_GATE:
                    self.start_node = Node(x, y)
                elif self.peta[y][x] == self.CELL_SLOT:
                    self.goal_nodes.append((x, y))

    def cetak_peta(self, rute=None, id_mobil=None):
        """
        Mencetak antarmuka visualisasi matriks ke CLI dengan format yang jelas.
        
        Args:
            rute (list): List koordinat (x,y) yang merupakan jalur optimal
            id_mobil (int): ID mobil yang sedang ditampilkan rute-nya
        """
        rute_set = set(rute) if rute else set()
        
        if id_mobil:
            print(f"\n--- VISUALISASI RUTE MOBIL #{id_mobil} ---")
        else:
            print("\n--- KONDISI DENAH PARKIR SAAT INI ---")
        
        last_pos = rute[-1] if rute else None

        for y, baris in enumerate(self.peta):
            baris_str = ""
            for x, sel in enumerate(baris):
                if last_pos is not None and (x, y) == last_pos:
                    baris_str += "X "
                elif (x, y) in rute_set:
                    baris_str += "~ "
                else:
                    baris_str += self.SIMBOL_MAP.get(sel, "?") + " "
            print(baris_str)
        
        print("-" * (self.kolom * 2 + 3))

    # ==========================================
    # MODUL 2: KECERDASAN BUATAN (BFS SOLVER)
    # ==========================================
    def _dalam_batas(self, x, y):
        """
        Validasi koordinat berada dalam batas matriks.
        
        Args:
            x, y (int): Koordinat yang dicek
            
        Returns:
            bool: True jika dalam batas
        """
        return 0 <= y < self.baris and 0 <= x < self.kolom

    def jalankan_bfs(self):
        """
        Algoritma Breadth-First Search untuk menemukan rute terpendek ke slot parkir.
        
        Returns:
            tuple: (rute, slot_koordinat) atau (None, None) jika tidak ada rute
        """
        if self.start_node is None or not self.goal_nodes:
            return None, None

        antrian = deque([self.start_node])
        dikunjungi = set([(self.start_node.x, self.start_node.y)])

        while antrian:
            node_sekarang = antrian.popleft()
            cx, cy = node_sekarang.x, node_sekarang.y

            # Uji Tujuan (Goal Test)
            if (cx, cy) in self.goal_nodes:
                rute = []
                temp = node_sekarang
                while temp:  # Backtracking untuk rekonstruksi path
                    rute.append((temp.x, temp.y))
                    temp = temp.parent
                return rute[::-1], (cx, cy) 

            # Ekspansi 4 Arah (Utara, Selatan, Barat, Timur)
            for dx, dy, _ in self.ARAH:
                nx, ny = cx + dx, cy + dy
                # Cek: dalam batas, belum dikunjungi, dan bukan obstacle/parked
                if (self._dalam_batas(nx, ny) and 
                    (nx, ny) not in dikunjungi and 
                    self.peta[ny][nx] not in (self.CELL_OBSTACLE, self.CELL_PARKED)):
                    antrian.append(Node(nx, ny, parent=node_sekarang))
                    dikunjungi.add((nx, ny))

        return None, None
    
    def hitung_statistik(self):
        """
        Hitung dan tampilkan statistik kondisi parkir saat ini.
        
        Returns:
            dict: Statistik dengan keys 'total_slot', 'slot_kosong', 'slot_terisi', 'persentase_terisi'
        """
        # Total slot harus menghitung slot kosong dan yang sudah terisi (CELL_SLOT + CELL_PARKED)
        total_slot = sum(1 for baris in self.peta for sel in baris if sel in (self.CELL_SLOT, self.CELL_PARKED))
        # Terisi berdasarkan status CELL_PARKED yang ada di peta saat ini
        terisi = sum(1 for baris in self.peta for sel in baris if sel == self.CELL_PARKED)
        kosong = total_slot - terisi

        persentase = (terisi / total_slot * 100) if total_slot > 0 else 0
        
        return {
            'total_slot': total_slot,
            'slot_kosong': kosong,
            'slot_terisi': terisi,
            'persentase_terisi': persentase
        }

    # ==========================================
    # MODUL 3: INTERFACE DEMO APLIKASI LANSUNG
    # ==========================================
    def reset_denah(self, nama_file_original=None):
        """
        Reset denah parkir ke kondisi semua slot kosong.

        Args:
            nama_file_original (str|None): Path file denah untuk menyimpan hasil reset.

        Returns:
            bool: True jika reset berhasil
        """
        try:
            if not self.peta:
                if self.peta_awal:
                    self.peta = [row.copy() for row in self.peta_awal]
                elif nama_file_original:
                    if not self.muat_denah(nama_file_original):
                        return False
                else:
                    print("[ERROR] Data denah tidak tersedia untuk reset!")
                    return False

            for y, row in enumerate(self.peta):
                for x, sel in enumerate(row):
                    if sel == self.CELL_PARKED:
                        self.peta[y][x] = self.CELL_SLOT

            # Sinkronkan atribut dimensi
            self.baris = len(self.peta)
            self.kolom = len(self.peta[0]) if self.peta else 0

            # Simpan perubahan ke file jika path disediakan
            if nama_file_original:
                self.simpan_denah(nama_file_original)

            return True
        except Exception as e:
            print(f"[ERROR] Gagal reset denah: {e}")
            return False
    
    def tampilkan_statistik(self):
        """Tampilkan statistik parkir dalam format yang user-friendly."""
        stats = self.hitung_statistik()
        print("\n" + "=" * 50)
        print("STATISTIK PARKIR SAAT INI")
        print("=" * 50)
        print(f"  Total Slot Parkir  : {stats['total_slot']}")
        print(f"  Slot Terisi        : {stats['slot_terisi']}")
        print(f"  Slot Kosong        : {stats['slot_kosong']}")
        print(f"  Tingkat Okupansi   : {stats['persentase_terisi']:.1f}%")
        print("=" * 50)

    def mode_demo_interaktif(self, nama_file):
        """
        Mode presentasi yang dikendalikan oleh user dengan input validation.
        
        Args:
            nama_file (str): Path file denah untuk simulasi
        """
        print("\n[SISTEM] Petunjuk: Tekan ENTER untuk mobil datang, ketik 'Q' untuk keluar,")
        print("         atau 'R' untuk reset denah ke kondisi awal.")
        
        id_mobil = 1
        while True:
            try:
                perintah = input(
                    f"\n[GERBANG] Mobil #{id_mobil} [ENTER=masuk / Q=keluar / R=reset]: "
                ).strip().lower()
                
                # Command: Quit
                if perintah == 'q':
                    print("\n[SISTEM] Mengakhiri simulasi parkir. Terima kasih!")
                    self.cetak_peta()
                    self.tampilkan_statistik()
                    break
                
                # Command: Reset
                elif perintah == 'r':
                    if self.reset_denah(nama_file):
                        self.cari_titik_penting()
                        self.riwayat_parkir = []
                        id_mobil = 1
                        print("\n[SISTEM] Denah telah di-reset ke kondisi awal!")
                        self.cetak_peta()
                    else:
                        print("\n[ERROR] Gagal reset denah!")
                    continue
                
                # Command: Enter (default - mobil masuk)
                elif perintah == "" or perintah == "enter":
                    pass  # Lanjut ke proses parkir
                else:
                    print("[PERINGATAN] Input tidak valid! Gunakan ENTER, Q, atau R.")
                    continue
                
                # Pindai ulang matriks untuk sync state
                self.cari_titik_penting()
                
                # Cek apakah ada slot kosong
                if not self.goal_nodes:
                    print(f"\n[SISTEM] Mobil #{id_mobil} ditolak!")
                    print("         Papan informasi menunjukkan: PARKIRAN PENUH!")
                    self.tampilkan_statistik()
                    break
                    
                # Jalankan BFS pathfinding
                rute, slot_ditemukan = self.jalankan_bfs()
                
                if rute:
                    panjang_langkah = len(rute) - 1
                    gx, gy = slot_ditemukan
                    print(f"\n[SUKSES] Rute ditemukan ke Slot #{len(self.riwayat_parkir) + 1}")
                    print(f"         Lokasi: P(x:{gx}, y:{gy}) | Jarak: {panjang_langkah} langkah")
                    
                    self.cetak_peta(rute, id_mobil)
                    
                    # Update state & persistensi
                    self.peta[gy][gx] = self.CELL_PARKED
                    self.riwayat_parkir.append({
                        'id': id_mobil,
                        'slot': (gx, gy),
                        'langkah': panjang_langkah,
                        'waktu': datetime.now().strftime("%H:%M:%S")
                    })
                    self.simpan_denah(nama_file)
                    
                    print(f"[UPDATE] Mobil #{id_mobil} berhasil parkir di P(x:{gx}, y:{gy})")
                    print(f"[SAVED] Denah parkir di file '{nama_file}' telah diperbarui.")
                    
                    id_mobil += 1
                else:
                    print(f"\n[GAGAL] Mobil #{id_mobil} TIDAK menemukan rute!")
                    print("         Alasan: Jalan menuju slot tersisa terblokir atau tidak ada slot.")
                    self.tampilkan_statistik()
                    break
                    
            except EOFError:
                # Handle EOF (end of stdin)
                print("\n[SISTEM] Input habis. Mengakhiri simulasi...")
                self.tampilkan_statistik()
                break
            except KeyboardInterrupt:
                # Handle Ctrl+C
                print("\n\n[SISTEM] Simulasi dihentikan oleh user (Ctrl+C). Terima kasih!")
                self.tampilkan_statistik()
                break
            except Exception as e:
                print(f"[ERROR] Kesalahan tidak terduga: {str(e)}")
                continue


# ===================================================================
# ENTRY POINT
# ===================================================================
if __name__ == "__main__":
    file_denah = "denah_parkir.txt"
    
    print("=" * 60)
    print("   SISTEM NAVIGASI PARKIR BFS — KELOMPOK 3")
    print("=" * 60)

    sistem = BFSParking()

    if sistem.muat_denah(file_denah):
        # Cetak kondisi awal peta sebelum ada mobil masuk
        sistem.cari_titik_penting()
        sistem.cetak_peta()
        sistem.tampilkan_statistik()
        
        # Mulai sesi live demo
        sistem.mode_demo_interaktif(nama_file=file_denah)
    else:
        print("[ERROR] Gagal menjalankan sistem. Periksa file denah!")