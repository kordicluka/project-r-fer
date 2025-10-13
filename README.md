# Implementacija nelinearnog modelskog prediktivnog upravljanja dinamičkim sustavom

[cite_start]**Sveučilište u Zagrebu Fakultet elektrotehnike i računarstva** [cite: 4]  
[cite_start]**Prijediplomski studij:** Računarstvo i Elektrotehnika i informacijske tehnologije [cite: 2]  
[cite_start]**Akademska godina:** 2025./2026. [cite: 7]

## Opis projekta

[cite_start]Cilj ovog projekta je razvoj i implementacija algoritma nelinearnog modelskog prediktivnog upravljanja (NMPC) za sustav kolica s njihalom u MATLAB okruženju[cite: 11]. [cite_start]Projekt obuhvaća matematičko modeliranje sustava, formulaciju i implementaciju NMPC algoritma za stabilizaciju njihala, te simulaciju i vizualizaciju ponašanja sustava u zatvorenoj petlji[cite: 12].

[cite_start]Cjelokupni kod bit će razvijen modularno unutar MATLAB klasa kako bi se osiguralo jasno razdvajanje funkcionalnosti i olakšala buduća nadogradnja[cite: 13].

## Članovi tima i zaduženja

| Student           | Zadatak i odgovornost                                                                    | Kontakt                                      |
| :---------------- | :--------------------------------------------------------------------------------------- | :------------------------------------------- |
| **Mateo Gnjidić** | [cite_start]Matematičko modeliranje i implementacija dinamičkog modela sustava [cite: 9] | [cite_start]`mateo.gnjidic@fer.hr` [cite: 9] |
| **Luka Kordić**   | [cite_start]Implementacija NMPC algoritma [cite: 9]                                      | [cite_start]`luka.kordic@fer.hr` [cite: 9]   |
| **Marko Maslać**  | [cite_start]Simulacija i vizualizacija ponašanja sustava [cite: 9]                       | [cite_start]`marko.maslac@fer.hr` [cite: 9]  |

**Mentor:** izv. prof. dr. sc. [cite_start]Branimir Novoselnik [cite: 6]

## Struktura repozitorija

[cite_start]Repozitorij prati predloženu strukturu za organizaciju koda i resursa[cite: 59]:

```
src/
├── model/
│   └── CartPendulumModel.m
├── controller/
│   └── NMPCController.m
└── simulation/
    └── SimulationEnvironment.m
results/
├── figures/
└── animations/
docs/
└── report.pdf
main.m
```

- [cite_start]`src/`: Direktorij s izvornim kodom podijeljenim na model, regulator i simulaciju[cite: 61, 62, 64, 66].
- [cite_start]`results/`: Direktorij za spremanje rezultata, grafova i animacija[cite: 68, 70, 71].
- [cite_start]`docs/`: Direktorij za projektnu dokumentaciju[cite: 72].
- [cite_start]`main.m`: Glavna skripta za pokretanje simulacije[cite: 74].

---
