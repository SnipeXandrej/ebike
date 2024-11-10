# Čo by som chcel naimplementovať:
### Štatistiky:
- [ ] Indikátor či je napájaný rotor
- [ ] Napätie na rotori
- [ ] Teplota motora
- [ ] Teplota baterky
- [ ] Otáčky motora (vypočítané z otáčok predného kolesa)
- [ ] Rýchlosť (vypočítané z otáčok motora)
- [ ] Okamžité napätie/prúd
- [ ] Okamžitá spotreba v kW
- [ ] Okamžitá spotreba Wh/1km
- [ ] Dlhodobá spotreba Wh/1km
- [ ] Výpočet zostavajúceho dosahu v kilometroch (z okamžitej spotreby Wh/1km)
- [ ] Výpočet zostavajúceho dosahu v kilometroch (z dlhodobej spotreby Wh/1km)
- [ ] Trip distance (celodobý)
- [ ] Trip distance (vyresetovaný po nejakej dobe neaktivity, dajme tomu že po 6 hodín)
- [ ] kWh použitých za celú dobu
- [ ] Baterkový počet cyklov (softvérový, samozrejme)
- [ ] Baterka napätie
- [ ] Baterka percentá (z využitia Ah za posledné nabitie)

### Elektronika:
- [x] Flipsky 75200
- [ ] Rotor (manuálne nastavovanie elektronickej prevodovky, ako manuál na motorke)
- [ ] Rotor (automatické nastavovanie elektronickej prevodovky, automat basically)
        1.  - možno sa bude lineárne/postupne znižovať napätie na rotori keď sa pôjde
              rýchlejšie a rýchlejšie
            - pri pomalších rýchlostiach, po rozbehnutí a nemeniacej sa rýchlosti (čiže cruising)
              znížiť napätie na rotori, aby toľko nežralo z baterky (algo: keď sa nebude prúdko meniť
              rýchlosť v km/h a nenastane prúdka zmenu plynu smerom hore)
- [ ] Obmedzovač/Prepínač výkonu (250W [aby bolo legálne], 1kW, a pod.)
- [ ] Svetlá
- [ ] Smerovky

### Konštrukcia:
- [ ] Vodotesná
- [ ] Výdrž baterky na 100km
- [x] RPi bootované z SSDčka (kvôli spolahlivosti)
