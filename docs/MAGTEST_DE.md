# Magazin-Trockenlauf auf ARES (2026-10-08)

Ziel (Samuel, 2026-10-08): die Bewegungen des UR auf ARES schnell am echten Aufbau testen und sehen, wie stabil der
Ablauf ist.

**Richtungen:** Der UR sitzt am **Heck** von ARES (in Fahrtrichtung), das Magazin zwischen UR und ARES-Mitte. Im
Projekt heißt die UR-Seite "vorne" (Mauerseite, `[ares] frame_x_points_to = "rear"`). Physisch liegt "vorne" in
diesem Dokument also **hinter dem Fahrzeug, über die UR-Kante hinaus**. Links und rechts sind hier immer in
Fahrtrichtung von ARES gemeint. Die große Simulation des ganzen Mauerbaus (RoboDK) folgt später. ARES fährt nicht, die Kamera wird nicht
benutzt, und kein Stein wird vorne abgesetzt.

Code: `mauer/magtest.py` (Ablauf), `tools/make_magtest.py` (Testdatei), `[magtest]` in `config/station.toml`
(Parameter), HMI wie bei einem Mauer-Job (`hmi/README.md`).

## Was passiert

Zwei Vollsteine wandern durch das Magazin. Ein **Zug** besteht aus:

1. Stein aus einem Magazinplatz greifen (echtes Schließen).
2. Mit dem Stein über eine Ablagepose der Mauer B fahren: 840 mm von der ARES-Mitte, also 280 mm über die
   UR-Kante (das Heck) hinaus.
3. Senkrecht absenken bis **50 mm über die echte Ablagehöhe** und **2 s halten** (Nachschwingen beobachten). Der
   Greifer bleibt **zu**, es gibt keinen Greiferbefehl.
4. Wieder hoch und den Stein in den nächsten Magazinplatz stellen (echtes Öffnen).

**Platzfolge ("Raupe"):** Zu Beginn stehen beide Steine gestapelt auf **r0y0** (Lage 1 + 2). Pro Schritt geht der
obere Stein auf Lage 1 der nächsten Position, der untere wird oben drauf gestellt (Lage 2). Die Positionen werden
in der Reihenfolge r0y0 → r0y1 → r0y2 → r1y2 → r1y1 → r1y0 und zurück durchlaufen. Jede der 6 Positionen wird in
Lage 1 und 2 gegriffen und belegt. Eine Runde hat 20 Züge, und danach stehen die Steine wieder auf r0y0.
Voreinstellung: 2 Runden = 40 Züge.

- r0 = die vom UR weiter entfernte Reihe (653,6 mm von der UR-Achse Richtung ARES-Mitte), r1 = die nähere
  (433,6 mm).
- y0 = links, y1 = Mitte, y2 = rechts (je 205 mm), in Fahrtrichtung gesehen.
- r1y0 hält im Mauer-Job Halbsteine. Sein Halter hat aber die 4 Kegel eines Vollsteins, deshalb benutzt der Test ihn
  mit einem Vollstein.
- Lage 3 bräuchte einen dritten Stein darunter und ist nicht dabei.

**Posen vorne:** Lagen 0 bis 3 und seitlich u = 0, −200, +200, −400, +400, −600, +600 mm (u < 0 = links in
Fahrtrichtung), **von innen nach außen**.
In Runde 1 geht es bis u = ±400. Die äußeren Posen u = ±600 (die kritischen für das Kippen, siehe die grobe Abschätzung
im README: ≈ 1 mm Reserve bei leerem Deck und u = ±800) kommen in Runde 2 bei den Zügen 21–26.
Lage 3 bei u = ±600 ist außer Reichweite (Hover-Pose 50 mm über der obersten Lage) und wird weggelassen.
`make_magtest` listet das.

## Vorbereitung

1. Konfiguration (Stand 2026-10-08, alles in `config/station.toml`):
   - `[ur5] mount_x = 360` (200 mm von der Kante am UR-Ende, physisch das Heck), `mount_y = 0`, `mount_z = 343.6`
     (Deck 333,6 + 10 mm Platte), `mount_rz = 90` (am Pendant geprüft).
   - `[ur] park_q_deg` ist für rz = 90 umgerechnet. Basisgelenk 71,83° statt 161,83°: dieselbe Pose auf ARES, nur
     10 mm höher.
   - `[ares] frame_x_points_to = "rear"`: Mauerläufe mit ARES-Fahrten sind gesperrt, bis das Layout gespiegelt ist.
     Der Magazin-Test fährt ARES nicht und ist davon nicht betroffen.
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
   - **2 Vollsteine gestapelt auf r0y0** (UR-ferne Reihe, links), alle anderen Plätze leer.
4. **Pendant-Prüfung von mount_rz** (2026-10-08 bestanden): Move-Tab, Feature "Base", Speed-Slider niedrig, auf
   die angezeigten Zahlen schauen, nicht auf die Pfeile. Wird **X größer**, fährt der TCP nach **rechts**. Wird
   **Y größer**, fährt er **vorwärts** zur ARES-Mitte. Fährt er anders, nicht starten.
5. Hinter ARES (über die UR-Kante hinaus) ist alles frei: bis etwa 0,7 m hinter dem Heck und ±0,8 m seitlich.
   Niemand steht im Arbeitsraum, der Not-Halt ist in der Hand.

## Ablauf in der HMI (Mauer-Tab)

1. **SIM zuerst:** SIM wählen, Szenario `none`, Prepare, Start. Die Simulation plant jede Gelenkbewegung mit dem
   Motion Guard wie am echten Roboter. Getestet 2026-10-08: 40/40 Züge, keine Verletzung, das Magazin steht am Ende
   wie am Anfang (≈ 25 s ohne Pausen).
2. **REAL:**
   - REAL wählen, **Step mode an**, Prepare / Connect. Geöffnet wird nur der UR, keine Kamera und keine ADS.
   - Die Preflight-Liste muss "REAL preflight ok" zeigen. Es bleibt die Warnung `[ur] payload_cog_mm PLACEHOLDER`.
     Sie blockiert nicht, wirkt aber auf das Nutzlastmodell des UR (Kollisionserkennung). Deshalb langsam fahren.
3. Start zeigt die Checkliste (Greifer leer, Steine auf r0y0 l1 + l2, Bereich hinter dem Heck frei, Pendant: X
   größer = rechts, Y größer = vorwärts, Not-Halt).
   Danach fragt die HMI jede Bewegung einzeln ab (3 pro Zug):
   - pick magazine slot …
   - dry place in front (u …, course …) …: "front" ist die UR-Seite, also hinter dem Fahrzeug
   - put down in magazine slot …
4. Den Speed-Slider am Pendant für die erste Runde niedrig lassen (z. B. 30 %). Die Parkpose ist für rz = 90 neu
   gerechnet: Die erste Fahrt in die Parkpose langsam beobachten.
5. Nach einigen sauberen Zügen kann man den Step mode ausschalten. Pause wirkt nach dem laufenden Zug, wenn der Stein
   abgestellt ist.

## Beobachten und notieren

- ARES beim Halten hinter dem Heck: Nachschwingen auf der Federung, Neigung, besonders bei u = ±600 (Züge 21–26)
  und in den tiefen Lagen.
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

CONFIRMED sind: Montagepunkt x / y / z / rz (rz am Pendant geprüft), UR am Heck (`frame_x_points_to`), Hover 50 mm,
Haltezeit 2 s, 2 Steine, Lagen 1 + 2.

Nicht getestet: Kamera, ARES-Fahrten, echtes Absetzen an der Mauer, Halbsteine, Magazinlage 3.
