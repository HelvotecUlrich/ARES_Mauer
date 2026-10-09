# Übergabe Testlauf 2026-10-09 (Kalibrierung auf ARES, dann Magazin-Trockenlauf)

Für die Session, die heute den Test am Roboter begleitet. Ablauf für den Bediener: `docs/ABLAUF_2026-10-09_DE.md`,
Details zum Steintest: `docs/MAGTEST_DE.md`, Kalibrierung: Testplan T4. Den echten Roboter bewegt nur Samuel.

## Stand (main, 2026-10-08 abends)

- UR auf ARES: `[ur5] mount_x 360, mount_z 343.6, mount_rz 90` (am Pendant bestätigt: X größer = TCP nach rechts,
  Y größer = vorwärts zur ARES-Mitte). Der UR sitzt am **Fahrzeugheck**; das Repo-Frame ist base_link um 180° gedreht
  (`[ares] frame_x_points_to = "rear"`). Echte Mauerläufe mit ARES-Fahrten sind deshalb gesperrt (preflight_real).
- Netz: Kamera, UR und ARES am Switch, Laptop-NIC "Ethernet" (192.168.1.20 / .56.20 / .50.1). Kamera per PoE,
  Hirose ab, UR-Werkzeugspannung per Skript auf 0 V (in der Installation speichern!). ARES: Ping ok, ADS-Port 48898
  antwortet NICHT (für heute nicht nötig).
- `calib_handeye orbit` prüft gegen ARES (Chassis, Controller-Kasten `[ares] controller_*`, Annahme).
- Magazin-Trockenlauf: `magtest` (baut `data/jobs/magtest.json` neu, öffnet die HMI); HMI-SIM 40/40 Züge.

## Vor dem Test am Roboter klären (Review 2026-10-08, bestätigte Funde)

1. **Magazin-Lage und Controller:** Die Magazinreihen liegen laut Konfiguration bei ARES-x −293,6 und −73,6 (Steine
   x −353,6 … −13,6), also auf der **Controller-Hälfte** jenseits der ARES-Mitte - nicht "zwischen UR und Mitte"
   (falsche Formulierung in station.toml / MAGTEST_DE). Der Controller-Kasten ist mit der Innenseite bei x −542
   modelliert (nur 18 mm über dem Deck). Liegt der Controller tatsächlich ~200 mm über dem Deck, steht er direkt neben
   Reihe r0 - dann `[ares] controller_*` korrigieren und den Magazin-Test neu bauen. Am Roboter nachsehen/messen.
2. **Kalibrierboard:** Die empfohlene Lage (400–500 mm von der UR-Achse) ist genau Magazinreihe r1 mit ihren Kegeln.
   Board dort nur auf eine flache Platte über den Kegeln legen oder an eine freie Deckstelle; **vor dem Magazin-Test
   wegnehmen** (der Motion Guard kennt das Board nicht).
3. **Schwenkbereich beim Magazin-Test:** Der Arm ragt mit Stein bis ~0,65 m (Kamera ~0,95 m) seitlich über BEIDE
   Fahrzeugseiten neben dem UR hinaus. ~1 m um die UR-Basis frei halten. Die HMI-Start-Checkliste nennt nur den
   Bereich hinter dem Heck (Text in `hmi/core/run_controller.py` start_checklist anpassen).
4. **HMI-Kleinigkeiten beim Dry Run ohne `--ares`:**
   - Grab/Live sind ohne Kamera aktiv und führen zu einem Fehler; einfach nicht benutzen.
   - HALT zeigt "HALT NOT sent: no ADS connection", obwohl der UR-Stopp gesendet wird.
   - Nach einem Fehler beim **Abstellen** kann der Stein schon im Zielplatz stehen; "Jaws empty" füllt aber den
     Startplatz im Modell wieder. Den Stein also immer zurück auf den Startplatz des Zugs stellen
     (Meldung / Run-Log `magtest_move` `slot_from`).
5. **Testplan T3 ist teils veraltet:** alte Parkpose 161,83°, Kamera am Werkzeugstecker, Messung zur "Vorderkante".
   Gültig sind `[ur] park_q_deg` (71,83°), PoE und MAGTEST_DE. Rotationskreis von ARES mit Controller: 864 mm statt 635.
6. `orbit` zeigt bei verworfenen Ansichten (ARES-Prüfung) eine falsche Zahl/Grenze in der Textzeile. Kosmetisch:
   verworfen ist verworfen.

Weitere Funde (Bodenführungen, Steinliste, Board-Tabellen, Planer) betreffen den Mauerbau, nicht den heutigen Test:
Journal `wf_f8838819-133` in der Session vom 2026-10-08.

## Parallel: andere Session

Eine zweite Session arbeitet im Worktree `/mnt/c/Users/samue/ARES_Mauer_wt/station2` an der Abholstation (zwei
Reihen, bis 4 Lagen) und an der RoboDK-Simulation des angepassten C. Sie ändert `config/station.toml`
(`[pickup_station]`) erst beim Zusammenführen in main. Ändert sich die Konfiguration während eines HMI-Laufs,
meldet der Preflight "config changed since the job was loaded": Release, `magtest` neu starten.
