# Magazin-Trockenlauf auf ARES (2026-10-08)

Ziel (Samuel, 2026-10-08): die Bewegungen des UR auf ARES schnell am echten Aufbau testen und sehen, wie stabil der
Ablauf ist. Die große Simulation des ganzen Mauerbaus (RoboDK) folgt später. ARES fährt nicht, die Kamera wird nicht
benutzt, und kein Stein wird vorne abgesetzt.

Code: `mauer/magtest.py` (Ablauf), `tools/make_magtest.py` (Testdatei), `[magtest]` in `config/station.toml`
(Parameter), HMI wie bei einem Mauer-Job (`hmi/README.md`).

## Was passiert

Zwei Vollsteine wandern durch das Magazin. Ein **Zug** besteht aus:

1. Stein aus einem Magazinplatz greifen (echtes Schließen).
2. Mit dem Stein nach vorne über eine Ablagepose der Frontwand B (840 mm vor der ARES-Mitte, also 280 mm vor der
   ARES-Vorderkante) fahren.
3. Senkrecht absenken bis **50 mm über die echte Ablagehöhe** und **2 s halten** (Nachschwingen beobachten). Der
   Greifer bleibt **zu**, es gibt keinen Greiferbefehl.
4. Wieder hoch und den Stein in den nächsten Magazinplatz stellen (echtes Öffnen).

**Platzfolge ("Raupe"):** Zu Beginn stehen beide Steine gestapelt auf **r0y0** (Lage 1 + 2). Pro Schritt geht der
obere Stein auf Lage 1 der nächsten Position, der untere wird oben drauf gestellt (Lage 2). Die Positionen werden
in der Reihenfolge r0y0 → r0y1 → r0y2 → r1y2 → r1y1 → r1y0 und zurück durchlaufen. Jede der 6 Positionen wird in
Lage 1 und 2 gegriffen und belegt. Eine Runde hat 20 Züge, und danach stehen die Steine wieder auf r0y0.
Voreinstellung: 2 Runden = 40 Züge.

- r0 = hintere Reihe (653,6 mm hinter der UR-Achse), r1 = vordere Reihe (433,6 mm).
- y0 = rechts (y = −205 mm), y1 = Mitte, y2 = links (+205 mm), in Fahrtrichtung gesehen.
- r1y0 hält im Mauer-Job Halbsteine. Sein Halter hat aber die 4 Kegel eines Vollsteins, deshalb benutzt der Test ihn
  mit einem Vollstein.
- Lage 3 bräuchte einen dritten Stein darunter und ist nicht dabei.

**Posen vorne:** Lagen 0 bis 3 und seitlich u = 0, −200, +200, −400, +400, −600, +600 mm, **von innen nach außen**.
In Runde 1 geht es bis u = ±400. Die äußeren Posen u = ±600 (die kritischen für das Kippen, siehe die grobe Abschätzung
im README: ≈ 1 mm Reserve bei leerem Deck und u = ±800) kommen in Runde 2 bei den Zügen 21–26.
Lage 3 bei u = ±600 ist außer Reichweite (Hover-Pose 50 mm über der obersten Lage) und wird weggelassen.
`make_magtest` listet das.

## Vorbereitung

1. Konfiguration (Stand 2026-10-08, alles in `config/station.toml`):
   - `[ur5] mount_x = 360` (200 mm hinter der Vorderkante), `mount_y = 0`, `mount_z = 343.6` (Deck 333,6 + 10 mm
     Platte), `mount_rz = 90` (Kabel nach hinten).
   - `[ur] park_q_deg` ist für rz = 90 umgerechnet. Basisgelenk 71,83° statt 161,83°: dieselbe Pose auf ARES, nur
     10 mm höher.
2. Testdatei bauen und HMI öffnen. Unter Windows im Repo-Ordner reicht **`magtest`**. Die Einzelschritte sind:
   ```
   py.exe tools/make_magtest.py            # data/jobs/magtest.json (--rounds 1: nur 20 Züge)
   py.exe -m hmi --job data/jobs/magtest.json
   ```
   `--ares` braucht es nicht, denn ARES wird nicht bewegt. Mit `--ares` läuft der ARES-Teil der HMI wie gewohnt mit.
3. Am Roboter:
   - UR eingeschaltet, Bremsen gelöst, kein Programm läuft.
   - Nutzlast am Pendant 1,68 kg.
   - Greifer **leer**.
   - **2 Vollsteine gestapelt auf r0y0**, alle anderen Plätze leer.
4. **Pendant-Prüfung von mount_rz:** Move-Tab, Feature "Base", Speed-Slider niedrig, kurz in **+X** tippen. Der TCP
   muss nach **ARES-links** fahren. Fährt er woandershin, nicht starten, denn dann stimmt `mount_rz` nicht.
5. Vor ARES ist alles frei: bis etwa 1,2 m vor der ARES-Mitte und ±0,8 m seitlich. Niemand steht im Arbeitsraum, der
   Not-Halt ist in der Hand.

## Ablauf in der HMI (Mauer-Tab)

1. **SIM zuerst:** SIM wählen, Szenario `none`, Prepare, Start. Die Simulation plant jede Gelenkbewegung mit dem
   Motion Guard wie am echten Roboter. Getestet 2026-10-08: 40/40 Züge, keine Verletzung, das Magazin steht am Ende
   wie am Anfang (≈ 25 s ohne Pausen).
2. **REAL:**
   - REAL wählen, **Step mode an**, Prepare / Connect. Geöffnet wird nur der UR, keine Kamera und keine ADS.
   - Die Preflight-Liste muss "REAL preflight ok" zeigen. Es bleibt die Warnung `[ur] payload_cog_mm PLACEHOLDER`.
     Sie blockiert nicht, wirkt aber auf das Nutzlastmodell des UR (Kollisionserkennung). Deshalb langsam fahren.
3. Start zeigt die Checkliste (Greifer leer, Steine auf r0y0 l1 + l2, Bereich frei, Pendant +X = links, Not-Halt).
   Danach fragt die HMI jede Bewegung einzeln ab (3 pro Zug):
   - pick magazine slot …
   - dry place in front (u …, course …) …
   - put down in magazine slot …
4. Den Speed-Slider am Pendant für die erste Runde niedrig lassen (z. B. 30 %). Die Parkpose ist für rz = 90 neu
   gerechnet: Die erste Fahrt in die Parkpose langsam beobachten.
5. Nach einigen sauberen Zügen kann man den Step mode ausschalten. Pause wirkt nach dem laufenden Zug, wenn der Stein
   abgestellt ist.

## Beobachten und notieren

- ARES beim Halten vorne: Nachschwingen auf der Federung, Neigung, besonders bei u = ±600 (Züge 21–26) und in den
  tiefen Lagen.
- Schutzstopps des UR (Nutzlast, Kollision), Greifen: sitzt der Stein danach sauber auf den Kegeln im Halter?
- Zeit pro Zug: Zeitstempel im Run-Log.
- Das Run-Log liegt in `data/runs/<Zeit>_hmi_real/run.jsonl` (`magtest_move`, `dry_placed`, `magtest_move_done`,
  Roboterbefehle), die Zusammenfassung in `hmi_summary.json` daneben.

## Wenn etwas schiefgeht

- **HALT** (Taste Leertaste / Esc oder Button) stoppt sofort. Der **Not-Halt** bleibt die Sicherheitsfunktion.
- **Fehler mitten im Zug** (Schutzstopp, HALT, Planung abgelehnt): Die HMI meldet "a stone may be in the jaws".
  1. Stein aus dem Greifer nehmen (Greifer am Pendant öffnen: DO0 kurz ein) oder aus dem Ablageplatz holen.
  2. Den Stein **auf den Platz "von" dieses Zuges zurückstellen**. Er steht in der Meldung und im Run-Log
     (`magtest_move`, `slot_from`).
  3. **Jaws empty**, dann **Resume**: Der Arm fährt erst in die Parkpose und wiederholt dann den Zug.
- Nach einem Neustart der HMI kennt sie den Stand nicht mehr. Steine zurück auf r0y0 (l1 + l2) stellen und neu
  starten.

## Abhängigkeiten (Ergebnis gilt unter diesen Annahmen)

Aus `data/jobs/magtest.json` (`depends_on`), 2026-10-08:
- `[tool] tcp_z` 147 (ASSUMPTION, Greiftiefe)
- `[brick] mass_kg` 3,0 (ASSUMPTION)
- `[wall] base_z` 4 (ASSUMPTION)
- `[deck] holder_z` 4 (ASSUMPTION)
- `[deck] magazine_rows_dx` / `magazine_y` (ASSUMPTION, Lage der Halter)
- `[ur] park_q_deg` (ASSUMPTION, "langsam prüfen")
- `[ur] payload_cog_mm` (PLACEHOLDER, nur Warnung)
- `[study] approach` 150 mm (ohne Tag)
- `[magtest] front_u_mm` / `front_courses` / `positions` / `rounds` (ASSUMPTION)

CONFIRMED sind: Montagepunkt x / y / z / rz, Hover 50 mm, Haltezeit 2 s, 2 Steine, Lagen 1 + 2.

Nicht getestet: Kamera, ARES-Fahrten, echtes Absetzen an der Mauer, Halbsteine, Magazinlage 3.
