# Čo by som chcel naimplementovať:
### Štatistiky:
- [ ] Rotor
    - [ ] Napätie na rotori
        - [ ] Naštelované na bicykli?
	- [ ] Indikátor či je napájaný rotor
	- [ ] Manuálne nastavovanie elektronickej prevodovky, ako manuál na motorke)
    - [ ] Automatické nastavovanie elektronickej prevodovky, automat basically)
            - možno sa bude lineárne/postupne znižovať napätie na rotori keď sa pôjde
              rýchlejšie a rýchlejšie
            - pri pomalších rýchlostiach, po rozbehnutí a nemeniacej sa rýchlosti (čiže cruising)
              znížiť napätie na rotori, aby toľko nežralo z baterky (algo: keď sa nebude prúdko meniť
              rýchlosť v km/h a nenastane prúdka zmenu plynu smerom hore)
	
- [ ] Teploty
    - [x] Teplota motora
        - [ ] Naštelované na bicykli?
    - [x] Teplota ESC
        - [ ] Naštelované na bicykli?

- [ ] Otáčky/trip distance
    - [x] Rýchlosť (vypočítané z otáčok motora)
        - [ ] Naštelované na bicykli?
    - [ ] Otáčky motora (vypočítané z otáčok predného kolesa)
        - [ ] Naštelované na bicykli?
    - [ ] Trip distance (celodobý)
    - [ ] Trip distance (vyresetovaný po nejakej dobe neaktivity, dajme tomu že po 6 hodín)

- [ ] Baterka
	- [ ] Meranie napätia
        - [ ] Naštelované na bicykli?
	- [ ] Meranie prúdu
        - [ ] Naštelované na bicykli?
    - [ ] Zobraziť okamžité napätie/prúd
	- [ ] Baterka percentá (z napätia)
    - [ ] Baterka percentá (z využitia Ah za posledné nabitie)
    - [ ] Baterkový počet cyklov (softvérový, samozrejme)
    - [ ] kWh použitých za celú dobu
    - [ ] Okamžitá spotreba v kW
    - [ ] Okamžitá spotreba Wh/1km
    - [ ] Dlhodobá spotreba Wh/1km
    - [ ] Výpočet zostavajúceho dosahu v kilometroch (z okamžitej spotreby Wh/1km)
    - [ ] Výpočet zostavajúceho dosahu v kilometroch (z dlhodobej spotreby Wh/1km)
    
- [ ] Ostatné
  - [ ] Vypínanie celého systému aby z baterky nič nežralo
  - [ ] Obmedzovač/Prepínač výkonu (250W [aby bolo legálne], 1kW, a pod.)
      - [ ] Naštelované na bicykli?
  - [ ] Svetlá
      - [ ] Naštelované na bicykli
  - [ ] Smerovky
      - [ ] Naštelované na bicykli?
  - [ ] Odomykanie (na kartičku? NFC?)
    
### Konštrukcia:
- [x] ESC: Flipsky 75200
- [ ] Vodotesné
- [ ] Výdrž baterky na 100km
- [x] RPi bootované z SSDčka (kvôli spolahlivosti)
