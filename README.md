# Čo by som chcel naimplementovať:
### Štatistiky:
- [ ] Rotor
    - [x] Menenie prúdu/napätia cez rotor pomocou PWM (zatiaľ statické, neregulované PWM)
      - [ ] TODO: Zbastliť obvod na riadenie MOSFETa
	- [x] Manuálne nastavovanie elektronickej prevodovky, ako manuál na motorke)
    - [ ] Automatické nastavovanie elektronickej prevodovky (automat basically)

          1. možno sa bude lineárne/postupne znižovať napätie na rotori keď sa pôjde rýchlejšie a rýchlejšie

          2. pri pomalších rýchlostiach, po rozbehnutí a nemeniacej sa rýchlosti (čiže cruising) znížiť napätie na rotori, aby toľko nežralo z baterky (algo: keď sa nebude prúdko meniť rýchlosť v km/h a nenastane prúdka zmenu plynu smerom hore)

<br><br>
- [ ] Teploty
    - [ ] Teplota motora
        - [ ] Naštelované na bicykli?
    - [ ] Teplota ESC
        - [ ] Naštelované na bicykli?

<br><br>
- [ ] Otáčky/trip distance
    - [ ] Rýchlosť (vypočítané z otáčok predného motora)
        - [ ] Naštelované na bicykli?
    - [ ] Rýchlosť (vypočítané z otáčok motora)
        - [ ] Naštelované na bicykli?
    - [ ] Trip distance (celodobý)
    - [ ] Trip distance (vyresetovaný po nejakej dobe neaktivity, dajme tomu že po 6 hodín)

<br><br>
- [ ] Baterka
	- [x] Meranie napätia
	- [x] Meranie prúdu
	- [x] Okamžitá spotreba vo Wattoch
	- [x] Baterka percentá (z napätia)
  - [ ] Baterka percentá (z využitia Ah za posledné nabitie)
  - [ ] Baterkový počet cyklov (softvérový, samozrejme)
  - [ ] kWh použitých za celú dobu
  - [ ] Okamžitá spotreba Wh/km
  - [ ] Dlhodobá spotreba Wh/km
  - [ ] Výpočet zostavajúceho dosahu v kilometroch (z okamžitej spotreby Wh/1km)
  - [ ] Výpočet zostavajúceho dosahu v kilometroch (z dlhodobej spotreby Wh/1km)

<br><br>
- [ ] Displej
  - [x] Zobraziť dátum a čas
  - [x] Zobraziť percento nabitia batérie
  - [x] Zobraziť napätie, prúd, výkon
  - [x] Zobraziť okamžitú spotrebu
  - [ ] Zobraziť dlhodobú spotrebu
  - [ ] Zobraziť indikátor či je rotor napájaný
  - [x] Zobraziť rýchlosť
  - [x] Zobraziť aktuálny stupeň výkonu a "stupeň" e-prevodovky
  - [x] Zobraziť "core execution time"
  - [x] Zobraziť uptime
  - [x] Zobraziť odometer
  - [x] Zobraziť trip

<br><br>
- [ ] Ostatné
  - [ ] Vypínanie celého systému aby z baterky nič nežralo
  - [x] Obmedzovač/Prepínač výkonu (250W [aby bolo legálne], 1kW, a pod.) (neni to ešte zcela dobre fungujúce)
      - [ ] Naštelované na bicykli?
  - [ ] Svetlá
      - [ ] Naštelované na bicykli
  - [ ] Smerovky
      - [ ] Naštelované na bicykli?
  - [ ] Odomykanie (na kartičku? NFC? 1-Wire?)
- [x] ESC: Flipsky 75200
- [x] EC:  ESP32
- [ ] Vodotesné
- [ ] Výdrž baterky na 100km
- [ ] Nabíjanie pomocou USB-C PD
- [ ] Nabíjanie cez vlastný spínaný zdroj
