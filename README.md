# MPPT---Fuzzy-Logic-Type-2
Maximum Power Point Tracking (MPPT) merupakan algoritma yang digunakan untuk pelacakan daya maksimum panel surya. 
Untuk membantu pelacakan daya maksimum oleh MPPT dibutuhkan sebuah metode pendukung, salah satunya adalah MPPT dengan metode AI.
Fuzzy Logic Type - 2 merupakan bagian dari MPPT metode AI
MPPT Fuzzy Logic Type 2 digunakan untuk pelacakan daya maksimum dengan output berupa duty cycle. 
MPPT Fuzzy Logic Type 2 memiliki nilai input berupa error dan delta error
Error merupakan pembagian antara (daya saat ini - daya sebelumnya) dengan (tegangan saat ini - tegangan sebelumnya)
Delta error merupakan pengurangan antara error saat ini dengan error sebelumnya
Dalam program ini metode Fuzzy Logic Type - 2 yang digunakan adalah metode sugeno
Perbedaan Fuzzy-Logic-Type-2 dan Fuzzy-Logic-Type-1 ialah Fuzzy-Logic-Type-2 memiliki 2 himpunan keanggotaan yaitu upper dan lower
Keunggulan dari Fuzzy-Logic-Type-2 ialah dapat mempercepat waktu pelacakan daya maksimum dengan hasil yang lebih akurat