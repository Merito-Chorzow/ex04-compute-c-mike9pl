# RAPORT — PID (Q15) z filtrem D i anti‑windup (back‑calculation)

## Opis algorytmu
- Sygnały i wzmocnienia w formacie Q15. Ts = 1.
- P: `P_k = Kp * e_k` (mnożenie Q15).
- I: `I_k = I_{k-1} + Ki * e_k`. Akumulator `i_acc` ograniczony do ±`i_limit` (clamping).
- D: `de = e_k − e_{k-1}`. LPF 1. rzędu: `d_filt[k] = α * d_filt[k-1] + (1 − α) * de`. `D = Kd * d_filt`.
- Anti‑windup: back‑calculation. Liczone po saturacji wyjścia: `diff = u_raw − u_sat`, korekcja `corr = aw_beta * diff` dodawana (odejmowana) do `i_acc`, potem ponowne ograniczenie `i_acc` do ±`i_limit`.
- Saturacja: `u = clamp(u_raw, u_min, u_max)`.

Legenda:

- P_k — wkład proporcjonalny w kroku k
- Kp, Ki, Kd — wzmocnienia P, I, D (Q15)
- e_k — błąd w kroku k (set − y_k)
- de — różnica błędu (e_k − e_{k-1})
- d_filt — przefiltrowana różnica błędu
- α — współczynnik filtra D (d_alpha)
- u_raw — niesaturacyjne wyjście sterujące w kroku k
- u_sat — wyjście sterujące po saturacji w kroku k
- u_min, u_max — granice saturacji sygnału sterującego
- i_acc — akumulator wkładu całkowego
- i_limit — limit akumulatora i_acc
- aw_beta — współczynnik anti‑windup (back‑calculation)  
- I_k — wkład całkowy w kroku k

## Warunki początkowe
set = 0.500, y0 = 0.000, u_min = -1.000, u_max = 1.000, i_limit = 1.000, d_alpha (PID) = 0.900

## Tabela metryk
- Metryki obliczone z programu `src/main.c` dla 1000 kroków.
- Kolumny: 
  
  Czas narastania = pierwszy k, dla którego y ≥ 0.9*set.

  Przeregulowanie [%] = max((y_max − set)/|set| *100,0).

  Błąd ustalony = średni |y-set| w ostatnich 50 krokach.

  Czas ustalenia = pierwszy k, od którego |y−set| ≤ 0.02*|set| dla reszty przebiegu.

| Konfiguracja | Kp | Ki | Kd | α (plant) | Czas narast. | Przeregul. [%] | Błąd ustal. | Czas ustal. | Uwagi |
|--|---:|---:|---:|---:|---:|---:|---:|---:|---|
|#1|0.300|0.000|0.000|0.05|- |0.00|0.3851|-  |brak stab.|
|#2|0.300|0.000|0.000|0.20|- |0.00|0.3847|-  |brak stab.|
|#3|0.600|0.050|0.000|0.05|42|1.73|0.0003|56 |stab.     |
|#4|0.600|0.050|0.000|0.20|53|0.00|0.0006|101|stab.     |
|#5|1.200|0.100|0.010|0.05|23|6.21|0.0003|71 |stab.     |
|#6|1.200|0.100|0.010|0.20|27|0.00|0.0003|56 |stab.     |

## Wnioski
1) Wpływ Kp/Ki/Kd:
- Zwiększenie Kp przyspiesza odpowiedź (krótszy czas narastania) ale zwiększa ryzyko przeregulowania i oscylacji. Wersje z dużym Kp były szybkie lecz mniej stabilne.
- Ki usuwa błąd stanu ustalonego, ale bez AW powoduje "napompowanie" całki podczas saturacji,  back‑calculation ogranicza ten efekt.
- Kd (w połączeniu z filtrem) tłumi przeregulowanie i redukuje oscylacje, ale zbyt duże Kd lub zbyt mały filtr (niskie α) może wprowadzać szum/niestabilność.

2) Rola filtra D (d_alpha):
- α bliskie 1 mocniej tłumi szum (mniejsze wahania sygnału D) kosztem opóźnienia sygnału różnicowego. W praktyce α≈0.9..0.99 daje kompromis: redukcja szumów bez zbytniego opóźnienia.
- D dobrze wyważone z filtrem obniża "przestrzelenie" i przyspiesza tłumienie oscylacji.

3) Sens i_limit i AW:
- Ograniczenie i_limit zapobiega zbyt dużemu wkładowi całki (praktyczne zabezpieczenie). Back‑calculation (aw_beta > 0) pomaga redukować integral windup gdy sygnał jest saturujący, dzięki czemu po odblokowaniu "akumulatora" układ szybko wraca do pożądanego stanu.
- Bez AW i z wąskim zakresem u_min/u_max całka może się "pompować" i powodować długie czasy powrotu.

## Deterministyczność
Symulacja jest deterministyczna: program używa deterministycznych operacji całkowitoliczbowych i brak w nim losowości ani zależnych od środowiska elementów. Powtórzenia tej samej konfiguracji i parametrów dają identyczne wyniki.