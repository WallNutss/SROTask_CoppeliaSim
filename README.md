# Tugas Sistem Sistem Robot Otonom

Tugas yang didapat dari Kelas Sistem Robot Otonom EE184945 merupakan simulasi bagaimana dapat mengontrol sebuah robot yang dipakai (P3DX) menggunakan simulasi aplikasi CoppeliaSim. Pada tugas ini diberikan 
6 tugas utama, pertama mulai dari pemahaman kode default yang diberikan pada aplikasi CoppeliaSim, Manual Control Robot, Object Follower, Localization and Routing, dan Moving through Multiple Waypoints.

## Penjelasan File
```
SROTask_CoppeliaSim
├── doc
├── SRO_ManucalControl.ttt
├── SRO_PathTracking.ttt
├── braintenberg.py
├── ddmr.py
├── p3dx_manualcontrol.py
├── remoteAPI.dll
├── sim.py
├── simConst.py
├── simpleTest.py
```

Pada folder, semua mode dan tugas ditaruh pada file p3dx_manualcontrol.py, sehingga untuk menjalankan program dapat kemudian dari direktori dasar jalankan
```
python p3dx_manualcontrol.py
```

dengan menjalankan simulasi dari robot tersebut dengan aplikasi CoppeliaSim yaitu file SRO_ManucalControl.ttt.


## Hasil
Task 2 : Manual Control
<p align="center">
  <img width="900" height="450" src="doc/srogif_task2.gif">
</p>

Task 3 : Object Follower
<p align="center">
  <img width="900" height="450" src="doc/sro_task3.gif">
</p>

Task 4 : Localization
<p align="center">
  <img width="900" height="450" src="doc/sro_task4.gif">
</p>

##  Reference
https://github.com/WallNutss/SROTask_CoppeliaSim
