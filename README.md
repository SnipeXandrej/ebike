# Screenshoty
<img src="https://raw.githubusercontent.com/SnipeXandrej/ebike/refs/heads/main/screenshots/MainTab.png" width="49.5%"/> <img src="https://raw.githubusercontent.com/SnipeXandrej/ebike/refs/heads/main/screenshots/MainTab2.png" width="49.5%"/>

<img src="https://raw.githubusercontent.com/SnipeXandrej/ebike/refs/heads/main/screenshots/AppTab.png" width="49.5%"/> <img src="https://raw.githubusercontent.com/SnipeXandrej/ebike/refs/heads/main/screenshots/EBIKETab.png" width="49.5%"/>

Ano, viem, tá terminológia okolo baterky je zlá... napríklad "Amphours Rated" neznamená koľko má ampérhodín od výroby, ale na koľko ampérhodín sa vie nabiť *teraz*. Niekedy opravím :)

# Čo by som chcel naimplementovať:

### Teploty
- [x] Teplota motora
- [ ] Teplota ESC

### Otáčky/trip distance
- [x] Rýchlosť (vypočítané z otáčok motora)
- [x] Odometer
- [x] Trip distance

### Baterka
- [x] Meranie napätia
- [x] Meranie prúdu
- [x] Okamžitá spotreba vo Wattoch
- [x] Baterka percentá (z napätia)
- [x] Baterka percentá (z využitia Ah za posledné nabitie)
- [ ] Baterkový počet cyklov (softvérový, samozrejme)
- [x] Okamžitá spotreba Wh/km
- [x] Dlhodobá spotreba Wh/km
- [x] Výpočet zostavajúceho dosahu v kilometroch (z dlhodobej spotreby Ah/1km)

### Displej
- [x] Zobraziť dátum a čas
- [x] Zobraziť percento nabitia batérie
- [x] Zobraziť napätie, prúd, výkon
- [x] Zobraziť okamžitú spotrebu
- [x] Zobraziť dlhodobú spotrebu
- [x] Zobraziť rýchlosť
- [x] Zobraziť aktuálny stupeň výkonu
- [x] Zobraziť "core execution time"
- [x] Zobraziť uptime
- [x] Zobraziť odometer
- [x] Zobraziť trip

### Ostatné
- [x] Vypínanie celého systému aby z baterky (takmer) nič nežralo
	* Zatial to je tak smíšno že to len vypína displej a možnosť ovládania motora
- [x] Obmedzovač/Prepínač výkonu (250W [aby bolo legálne], 1kW, a pod.)
- [ ] Svetlá
	- [x] Predné
	- [ ] Zadné
	- [ ] Smerovky
- [ ] Odomykanie (na kartičku? NFC? 1-Wire?)
- [x] ESC: Flipsky 75200
- [x] EC:  ESP32
- [x] Telefón ako displej
- [ ] Vodotesné
- [x] Výdrž baterky na 100km (pri sústavnej rýchlosti cca 30-35km/h)
- [ ] Nabíjanie pomocou USB-C PD
- [ ] Nabíjanie cez vlastný spínaný zdroj
- [ ] USB-C PD 65W + USB-A porty
