# Zwift Ventilátor Vezérlő – ESP32-C3 BLE/WiFi

Automatikus ventilátor vezérlő rendszer Zwift (vagy más indoor edzésprogram) használatához, **Seeed XIAO ESP32-C3** mikrovezérlőre alapozva. A rendszer BLE-n (Bluetooth Low Energy) keresztül olvassa a pulzus- vagy teljesítménymérő adatait, és ennek megfelelően automatikusan szabályozza a ventilátorokat több fokozatban.

---

## Tartalomjegyzék

1. [Áttekintés](#1-áttekintés)
2. [Hardver](#2-hardver)
3. [Projekt struktúra](#3-projekt-struktúra)
4. [Függőségek](#4-függőségek)
5. [Első beüzemelés](#5-első-beüzemelés)
6. [BLE érzékelő csatlakozás](#6-ble-érzékelő-csatlakozás)
7. [Ventilátor vezérlés és zónák](#7-ventilátor-vezérlés-és-zónák)
8. [WiFi funkciók és webes felület](#8-wifi-funkciók-és-webes-felület)
9. [WebSerial parancsok](#9-webserial-parancsok)
10. [LED állapotjelzés](#10-led-állapotjelzés)
11. [Nyomógomb funkciók](#11-nyomógomb-funkciók)
12. [Watchdog és Deep Sleep](#12-watchdog-és-deep-sleep)
13. [EEPROM – Beállítások mentése](#13-eeprom--beállítások-mentése)
14. [Dual-core működés](#14-dual-core-működés)
15. [Tipikus használati folyamat](#15-tipikus-használati-folyamat)

---

## 1. Áttekintés

A projekt célja, hogy Zwift vagy más indoor kerékpáros/futó edzésprogram használata közben automatikusan irányítsa a ventilátorokat a sportoló felé, a pillanatnyi terhelés (pulzus vagy watt) alapján. A rendszer:

- **BLE kliensként** csatlakozik a pulzus- vagy teljesítménymérőhöz,
- az aktuális értéket **zónákba sorolja**,
- a zónának megfelelő **relét kapcsol**, ezzel 3 ventilátor fokozatot és egy külső aljzatot vezérel,
- **webes dashboardon** keresztül manuálisan is kezelhető,
- **deep sleep** módba lép, ha nincs adat (edzés vége).

---

## 2. Hardver

| Alkatrész | GPIO | Leírás |
|-----------|------|--------|
| Seeed XIAO ESP32-C3 | – | Mikrovezérlő |
| Relé 1 – 1. fokozat | GPIO 10 | 1. ventilátor fokozat |
| Relé 2 – 2. fokozat | GPIO 9 | 2. ventilátor fokozat |
| Relé 3 – 3. fokozat | GPIO 8 | 3. ventilátor fokozat |
| Külső aljzat relé | GPIO 2 | `relayOutlet` – extra eszköz |
| Relé engedélyező pin | GPIO 21 | `relayEN` |
| WiFi/BLE állapot LED | GPIO 5 | `LED_wifi` |
| EEPROM hiba LED | GPIO 4 | `LED_eeprom` |
| Nyomógomb | GPIO 3 | `WAKEUP_PIN` – ébresztés, vezérlés |

A PWM csatorna (5000 Hz, 8 bit) a pillanatnyi teljesítmény vizuális visszajelzésére szolgál.

---

## 3. Projekt struktúra

```
├── platformio.ini                            # PlatformIO konfiguráció
├── src/
│   └── esp32c3_seeed_xiao_vent_v4.cpp        # Fő programkód
├── data/                                     # SPIFFS fájlrendszer
│   ├── index.html                            # Webes dashboard
│   ├── wifimanager.html                      # WiFi Manager oldal
│   ├── style.css                             # WiFi Manager stíluslap
│   ├── style2.css                            # Dashboard stíluslap
│   ├── script.js                             # Dashboard WebSocket JavaScript
│   ├── ssid.txt                              # Mentett WiFi SSID
│   ├── pass.txt                              # Mentett WiFi jelszó
│   ├── ip.txt                                # Mentett IP-cím
│   └── gateway.txt                           # Mentett gateway
└── lib/                                      # Külső könyvtárak
    ├── WebSerial/
    ├── ESPAsyncWebServer/
    ├── OneButton/
    └── ...
```

---

## 4. Függőségek

A projekt **PlatformIO** és **Arduino framework** alapú.

- **Platform:** `espressif32`
- **Board:** `seeed_xiao_esp32c3`
- **Partíciós tábla:** `min_spiffs.csv`

Felhasznált könyvtárak:

| Könyvtár | Funkció |
|----------|---------|
| BLE | Bluetooth Low Energy kliens |
| WiFi | WiFi csatlakozás |
| AsyncTCP | Aszinkron TCP alap |
| ESPAsyncWebServer | Webes szerver és WebSocket |
| WebSerial | Böngészőalapú soros konzol |
| EEPROM | Beállítások tárolása |
| ElegantOTA | OTA firmware frissítés |
| TickTwo | Szoftveres timer (watchdog) |
| OneButton | Nyomógomb kezelés |
| Arduino_JSON | JSON feldolgozás |
| SPIFFS | Fájlrendszer (WiFi konfiguráció) |

---

## 5. Első beüzemelés

1. A firmware feltöltéséhez nyisd meg a projektet **PlatformIO**-ban, majd töltsd fel az ESP32-C3-ra.
2. A SPIFFS fájlrendszer feltöltéséhez használd a PlatformIO „Upload Filesystem Image" funkcióját (`data/` mappa tartalma).
3. **Első bekapcsoláskor** (ha nincs mentett WiFi konfiguráció) az ESP32 Access Point módban indul:
   - Hálózat neve: `ESP-WIFI-MANAGER`
   - Csatlakozás után böngészőből add meg az SSID-t, jelszót, IP-címet és gatewayt.
   - A beállítások a SPIFFS-be mentődnek (`ssid.txt`, `pass.txt`, `ip.txt`, `gateway.txt`).
4. WiFi csatlakozás után a webes dashboard az eszköz IP-címén érhető el.

---

## 6. BLE érzékelő csatlakozás

A program BLE kliensként automatikusan keres és csatlakozik egy kompatibilis érzékelőhöz:

| Érzékelő típus | BLE Szolgáltatás | Karakterisztika | `erzekelo` érték |
|----------------|-----------------|-----------------|-----------------|
| Pulzusmérő | Heart Rate Service `0x180D` | `0x2A37` | `111` |
| Teljesítménymérő | Cycling Power Service `0x1818` | `0x2A63` | `222` |

Az érzékelő típusa **WebSerial-on** keresztül állítható:
- `erzekeloheart` – pulzusmérő mód
- `erzekelopower` – teljesítménymérő mód
- `milyenerzekelo` – aktuális mód lekérdezése

### Adatfeldolgozás

A `notifyCallback` fogadja az érzékelő értesítéseit:
- **Pulzusnál:** `pData[1]` (bpm)
- **Wattnál:** `(pData[3] << 8) | pData[2]` (watt)

Az `atlagolas()` függvény az utolsó néhány mérés átlagát számítja ki, csökkentve a mérési zajt.

---

## 7. Ventilátor vezérlés és zónák

A `ventillatorvezerles()` függvény az aktuális `teljesitmeny` (pulzus vagy watt) értéke alapján zónákba sorol, és a megfelelő reléket kapcsolja.

### Zónák

| Zóna | Feltétel | Aktív relé | Leírás |
|------|----------|------------|--------|
| 0 | `teljesitmeny == 0` VAGY `< alapteljesitmeny` | Minden ki | Nincs terhelés |
| 1 | `alapteljesitmeny < teljesitmeny < ZONE_1` | GPIO 10 | 1. fokozat |
| 2 | `ZONE_1 ≤ teljesitmeny < ZONE_2` | GPIO 9 | 2. fokozat |
| 3 | `ZONE_2 ≤ teljesitmeny < sprintzona` | GPIO 8 | 3. fokozat |
| 4 (Sprint) | `teljesitmeny ≥ sprintzona` | GPIO 8 | Sprint (késleltetéssel) |

### Zónahatárok beállítása

**Pulzusmérő esetén** – `mymaxheartrate(szám)` paranccsal:
- ZONE_1 = maxHR × 0,75
- ZONE_2 = maxHR × 0,85
- sprintzona = maxHR × 0,96

**Teljesítménymérő esetén** – `myftp(szám)` paranccsal:
- ZONE_1 = FTP × 0,75
- ZONE_2 = FTP × 0,90
- sprintzona = FTP × 1,18
- alapteljesitmeny = 20 W

### Késleltetések

A `relek()` függvény az egyes zónaváltásoknál beállítható késleltetést alkalmaz:

| Változó | Leírás |
|---------|--------|
| `kesleltetes0` | Késleltetés a 0. zónánál |
| `kesleltetes1` | Késleltetés az 1. zónánál |
| `kesleltetes2` | Késleltetés a 2. zónánál |
| `kesleltetes3` | Késleltetés a 3. zónánál |
| `kesleltetessprint` | Késleltetés sprint zónánál |
| `kesleltetesend` | Késleltetés leállásnál |

---

## 8. WiFi funkciók és webes felület

### WiFi Manager

Ha nincs mentett WiFi konfiguráció, az ESP32 AP módban indul (`ESP-WIFI-MANAGER`). A `wifimanager.html` oldalon megadható:
- WiFi SSID és jelszó
- Statikus IP-cím és gateway

### Webes Dashboard (`index.html`)

A dashboard az eszköz IP-címén érhető el. Funkciók:
- A 3 ventilátor fokozat **toggle gombokkal** kapcsolható be/ki manuálisan.
- A külső aljzat (`relayOutlet`) szintén kapcsolható.
- **WebSocket**-en keresztül valós idejű állapotfrissítés.

### WebSerial (`/webserial`)

Böngészőből parancsok küldhetők az eszköznek (lásd a [WebSerial parancsok](#9-webserial-parancsok) fejezetet).

### ElegantOTA (`/update`)

Firmware frissítés böngészőn keresztül (OTA – Over The Air), a `/update` útvonalon.

---

## 9. WebSerial parancsok

A `recvMsg()` függvény az alábbi parancsokat értelmezi. A WebSerial a böngészőből érhető el: `http://<IP>/webserial`

### Általános parancsok

| Parancs | Leírás |
|---------|--------|
| `help` | Elérhető parancsok listája |
| `reset` | EEPROM alapértékek visszaállítása, majd újraindítás |
| `wifireset` | WiFi beállítások törlése, AP mód visszaállítása |
| `reboot` | Újraindítás |
| `off` | Deep sleep mód aktiválása |

### Ventilátor vezérlés

| Parancs | Leírás |
|---------|--------|
| `venton` | Minden ventilátor be |
| `ventoff` | Minden ventilátor ki |
| `vent1on` / `vent1off` | 1. fokozat manuális be/ki |
| `vent2on` / `vent2off` | 2. fokozat manuális be/ki |
| `vent3on` / `vent3off` | 3. fokozat manuális be/ki |

### Érzékelő beállítás

| Parancs | Leírás |
|---------|--------|
| `erzekeloheart` | Pulzusmérő módra váltás |
| `erzekelopower` | Teljesítménymérő módra váltás |
| `milyenerzekelo` | Aktuális érzékelő típus lekérdezése |
| `mymaxheartrate(szám)` | Max pulzus beállítása, zónák automatikus számítása |
| `myftp(szám)` | FTP beállítása, zónák automatikus számítása |

### Zóna beállítások

| Parancs | Leírás |
|---------|--------|
| `elsozona(szám)` | 1. zóna határ kézi beállítása |
| `masodikzona(szám)` | 2. zóna határ kézi beállítása |
| `sprintzona(szám)` | Sprint zóna határérték |
| `alapteljesitmeny(szám)` | Alap teljesítmény küszöb (watt vagy bpm) |
| `zonak?` | Aktuális zónahatárok kiírása |

### Késleltetés beállítások

| Parancs | Leírás |
|---------|--------|
| `kesleltetesnulla(ms)` | Késleltetés a 0. zónánál (ms) |
| `kesleltetesegy(ms)` | Késleltetés az 1. zónánál (ms) |
| `kesleltetesketto(ms)` | Késleltetés a 2. zónánál (ms) |
| `kesleltetesharom(ms)` | Késleltetés a 3. zónánál (ms) |
| `kesleltetessprint(ms)` | Késleltetés sprint zónánál (ms) |
| `kesleltetesend(ms)` | Késleltetés leállásnál (ms) |
| `kesleltetesek?` | Aktuális késleltetések kiírása |

### Speciális üzemmódok

| Parancs | Leírás |
|---------|--------|
| `run` | Teszt mód indítása |
| `teszt` | Teszt mód kikapcsolása |
| `hutesuzemmodbe` | Hűtés üzemmód bekapcsolása |
| `hutesuzemmodki` | Hűtés üzemmód kikapcsolása |
| `kalibralasbe` | Kalibrálás mód bekapcsolása |
| `kalibralaski` | Kalibrálás mód kikapcsolása |
| `lcdon` / `lcdoff` | LCD kezelés (előkészítve, jövőbeli funkció) |

---

## 10. LED állapotjelzés

A `fct_ledUpdate()` függvény a `LED_wifi` (GPIO 5) pinen villogási mintával jelzi az aktuális állapotot:

| Minta | Frekvencia | Állapot |
|-------|-----------|---------|
| Gyors villogás | 50 ms | Nincs WiFi, nincs BLE kapcsolat |
| Közepes villogás | 300 ms | WiFi OK, BLE nincs csatlakozva |
| Lassú villogás | 900 ms | BLE csatlakozva, rendszer aktív |

---

## 11. Nyomógomb funkciók

A GPIO 3-as nyomógomb (OneButton könyvtárral kezelve) többféle műveletet támogat:

| Művelet | Hatás |
|---------|-------|
| Egyszeri kattintás | Watchdog számláló reset |
| Dupla kattintás | WiFi beállítások törlése, újraindítás |
| Hosszú nyomás (elengedésre) | Deep sleep mód (minden relé ki) |

---

## 12. Watchdog és Deep Sleep

### Watchdog

A `TickTwo watchDOG` szoftveres timer másodpercenként meghívja a `fct_Watchdog()` függvényt, amely figyeli a `watchdogCounter` értékét:

- Ha `teljesitmeny > 0` → `fct_WatchdogReset()` hívása, számláló nullázódik.
- Ha `teljesitmeny == 0` és a számláló eléri a `timetosleep` értéket → minden relé kikapcsol → **deep sleep** aktiválódik.

### Deep Sleep

- Az ESP32 gombnyomásra (GPIO 3) ébreszthető fel.
- A `fct_goToSleep()` boot utáni állapotot is vizsgál: ha az ébredés nem gombnyomásból történt (pl. reset), az eszköz visszamegy deep sleep-be.
- A `bootCounter` mechanizmus gondoskodik arról, hogy a teszt-, hűtés- és kalibrálás üzemmódok legfeljebb 2 bootciklus után automatikusan kikapcsoljanak.

---

## 13. EEPROM – Beállítások mentése

Minden fontos paraméter EEPROM-ba mentődik, így újraindítás után is megmaradnak:

- Zónahatárok (`ZONE_1`, `ZONE_2`, `sprintzona`, `alapteljesitmeny`)
- Késleltetések (`kesleltetes0`–`kesleltetes3`, `kesleltetessprint`, `kesleltetesend`)
- Érzékelő típusa (`erzekelo`)
- Üzemmód jelzők (teszt, hűtés, kalibrálás)

Az `eeprom_check()` és `eeprom_valid()` függvények validálják az EEPROM tartalmát, és szükség esetén alapértékekre állítják a paramétereket.

---

## 14. Dual-core működés

Az ESP32 két CPU magját a rendszer párhuzamosan használja:

| Task | Funkció |
|------|---------|
| `loop()` (Core 1) | BLE adatfeldolgozás, ventilátor vezérlés |
| `loop2()` (Core 0, Task) | OTA frissítés, WebSocket karbantartás, watchdog update, LED frissítés, gombkezelés |

---

## 15. Tipikus használati folyamat

```
1. Első bekapcsolás
   └─► AP mód indul: "ESP-WIFI-MANAGER"
       └─► Böngészőből WiFi beállítás

2. WiFi csatlakozás után
   └─► Webes dashboard: http://<IP>/
   └─► WebSerial: http://<IP>/webserial
   └─► OTA frissítés: http://<IP>/update

3. Érzékelő és zóna beállítás (WebSerial-on):
   └─► mymaxheartrate(190)  – vagy –  myftp(250)

4. BLE automatikus csatlakozás
   └─► Pulzusmérő vagy teljesítménymérő kerékpár/futópad

5. Edzés közben
   └─► Ventilátorok automatikusan váltanak fokozatot a zónák alapján
   └─► LED lassan villog (BLE csatlakozva)

6. Edzés vége (nincs adat)
   └─► Watchdog visszaszámlál
   └─► Relék kikapcsolnak
   └─► Deep sleep aktiválódik

7. Felébresztés
   └─► Nyomógomb (GPIO 3) egyszeri megnyomása
```

---

## Licenc

A projekt személyes és oktatási célra szabadon felhasználható.
