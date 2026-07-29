import os
import shutil

def ubah_ekstensi_dan_pindah(folder_asal, folder_tujuan):
    # Memastikan folder tujuan sudah ada, jika belum maka akan dibuat otomatis
    if not os.path.exists(folder_tujuan):
        os.makedirs(folder_tujuan)
        print(f"Folder tujuan tidak ditemukan. Membuat folder: {folder_tujuan}")

    # Membaca semua file di dalam folder asal
    files = os.listdir(folder_asal)
    jumlah_terkonversi = 0

    for file_name in files:
        # Memeriksa apakah file memiliki ekstensi .py
        if file_name.endswith('.py'):
            # Menentukan path lengkap file asal
            path_asal = os.path.join(folder_asal, file_name)
            
            # Mengganti ekstensi file dari .py menjadi .txt
            nama_baru = file_name[:-3] + '.txt'
            # Menentukan path lengkap file tujuan
            path_tujuan = os.path.join(folder_tujuan, nama_baru)
            
            # Menyalin file ke folder baru dengan nama dan ekstensi baru
            shutil.copy(path_asal, path_tujuan)
            print(f"Berhasil menyalin & mengubah: {file_name} -> {nama_baru}")
            jumlah_terkonversi += 1

    print("---")
    print(f"Selesai! Total {jumlah_terkonversi} file .py berhasil diubah ke .txt dan dipindahkan.")

# === ISI JALUR FOLDER DI SINI ===
# Gunakan 'r' di depan string untuk menghindari error backslash (\) di Windows
FOLDER_SUMBER = r"C:\jalur\ke\folder\asal"
FOLDER_HASIL = r"C:\jalur\ke\folder\tujuan"

# Menjalankan fungsi