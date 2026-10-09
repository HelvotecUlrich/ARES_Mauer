# Layout „final“ (2026-10-09)

Das C mit Tür und Fenster und die Abholstation mit zwei Reihen, wie am 2026-10-09 simuliert und mit Samuel
festgelegt. Dieser Ordner ist ein **eingefrorener Stand**, Git-Tag `layout-final`. Die Dateien sind Kopien der
erzeugten Ausgaben (`results/`, `targets/guides/`). Ändert sich die Konfiguration, werden diese neu erzeugt, dieser
Ordner aber nicht.

## Das Layout

**Mauer** (`[[wall.legs]]`, Fotos `input/`): 4 Lagen, 67 Steine (53 Voll, 14 Halb), gebaut B → A → C in 4 Halten.

| Schenkel | Länge | Lage zu ARES | Besonderheit |
|---|---|---|---|
| B | 9 Steine (1,8 m) | vorne, 840 mm | läuft durch beide Ecken, wird zuerst gebaut |
| A | 5 Steine (1,0 m) | rechts, 580 mm | Fenster in Lage 3 + 4, 400 mm breit, 300 mm von beiden Enden; von der Ecke zu B aus gebaut |
| C | 5 Steine (1,0 m) | links, 580 mm | nur 400 mm am freien Ende, daneben 600 mm Tür an B |

**Abholstation** (`[pickup_station]`): auf dem Boden, eine MDF-Platte mit zwei Reihen im Abstand von 200 mm. Die Codes
S0 / S1 liegen im vorderen Streifen (an ARES) bei ±500 mm. Stapelhöhen so, dass der UR jeden Stein auch bei
Andockfehlern bis 30 mm / 1,5° sicher greift:

| Reihe | Stapel (Gravur auf der Platte, von links) | Lagen |
|---|---|---|
| 1 (vorne, an ARES) | h00 Halb · s00 · s01 · s02 · s03 · s04 · h01 Halb · h02 Halb · h03 Halb | 2 · 4 · **5 · 5 · 5 · 5** · 4 · 4 · 2 |
| 2 (hinten) | s05 · s06 · s07 · s08 · s09 · s10 | 2 · 4 · 4 · 4 · 4 · 2 |

= **56 Steine (44 Voll + 12 Halb)**. Dazu das Magazin auf ARES mit 12 Voll + 2 Halb: **alle Steine für das C auf
einmal**, 4 Stationsfahrten, kein Nachlegen, am Ende bleiben 3 Vollsteine übrig. **Jeden Stapel genau so hoch wie in
der Tabelle füllen.** Ein Stein darüber ist der Software unbekannt und liegt den Greifbacken im Weg.

## Dateien

| Datei | Was |
|---|---|
| `steinliste.pdf` | Steine je Schenkel / Lage, Beladeplan, **Stapelhöhen der Station**, Setzreihenfolge |
| `floor_map.pdf` | Bodenplan 1:20 auf A3 (in 100 % drucken, 1-m-Balken prüfen), Seite 2: Messpunkte, Diagonalen, Teile |
| `floor_plan.md` | dieselben Koordinaten und Diagonalen als Text |
| `laser/laser_sheet_1..5.dxf` | MDF **5 mm** 800 × 600 (seit 2026-10-09, vorher 4 mm): Blatt 1 + 2 Station, 3 – 5 Bodenführungen (Ebenen CUT / ENGRAVE / SHEET) |
| `laser/peg_fit_test.dxf`, `laser/laser_test.dxf` | **zuerst** im 5-mm-MDF lasern und prüfen (Schnittbreite und Zapfenloch wurden in 4 mm eingemessen) (Ablauf: `targets/guides/README.md`) |
| `laser/locating_cone.stl` | Zentrierkegel, 64 Stück + Reserve (Bambu X1E, liegt schon in Druckrichtung) |
| `laser/laser_sheets.png` / `.svg` | Vorschau der Blätter |
| `l_wall_sim.md` | Bericht der RoboDK-Kollisionssimulation |
| `l_wall_plan.md` | Plan: Halte, Fahrwege, Boards, Station (rein rechnerisch) |
| `bilder/` | RoboDK-Ansichten (Station, Draufsicht, Ecke, letzter Halt) und die Layout-Zeichnung |
| `l_wall_sim_final.mp4` | Video der Simulation (nur lokal, nicht in Git: 25 MB) |

## Bewegungen (Stand 2026-10-09 nachmittags)

- **Halbsteine 20 mm höher greifen** (`[half_brick] grasp_above_top_mm`, ASSUMPTION): Das Pin-Paar eines Halbsteins steht
  in der Mitte unter dem Greifer. Alle Halbstein-Posen in Magazin, Station und Wand sind entsprechend angehoben.
- **Seitlich setzen** (`[ur] place_side_mm` 20 mm, `place_side_above_pins_mm` 30 mm): Ein Stein mit genau einem schon
  gesetzten Nachbarn in seiner Lage (auch über die Ecke) kommt 20 mm neben seiner Endlage herunter, weg vom Nachbarn,
  bis seine Unterkante 30 mm über den Pins der unteren Lage ist. Dann fährt er seitlich an den Nachbarn und erst danach
  nach unten. Das betrifft 57 von 67 Steinen. Ohne Nachbarn oder mit Nachbarn auf beiden Seiten geht es senkrecht nach
  unten wie bisher.
- **Kürzester freier Umweg**: Ist der direkte Weg blockiert, nimmt der Motion Guard den kollisionsfreien Umweg mit dem
  kleinsten Gelenkweg, nicht automatisch die Parkpose. Im ganzen C sind das rund 6 % weniger Armweg.

Bericht, Bilder und Video sind von diesem Stand (Branch `station2`). Das Tag `layout-final` markiert die Festlegung des
Layouts, und das Layout selbst hat sich seitdem nicht geändert.

## Geprüft

- RoboDK (`robodk/simulate.py --animate --video`, mit seitlichem Setzen und Halbstein-Griff): 67/67 Steine und 53/53
  Umlagerungen Station → Magazin mit Kollisionsprüfung, 11 ARES-Fahrten ohne Kollision, 0 Mal Nachlegen.
- Weltsimulation mit Motion Guard (wie die HMI-SIM; realistische Fahr-, Andock- und Messfehler), Seeds 1 – 3:
  je 67/67 Steine gesetzt, keine Verletzung, maximal 2,1 – 2,6 mm Abweichung, 50 – 52 ARES-Fahrten.

## Hängt ab von (nicht bestätigt)

Lage des UR-Controllers (`[ares] controller_*`), Stationslage und -aufbau und Andocktoleranz `dock_tol_deg`
(ASSUMPTION), Maße des Halbsteins (PLACEHOLDER), Werkzeug-TCP. Vollständige Liste: `l_wall_sim.md`, Abschnitt
„Depends on“.

## Offen

- Standfestigkeit der 5er-Stapel (0,6 m hoch) ist nicht geprüft.
- Echte Mauerläufe sind gesperrt, bis die ARES-Fahrten gespiegelt sind (der UR sitzt am Heck,
  `[ares] frame_x_points_to = "rear"`).
- Der RoboDK-geprüfte Job (`data/jobs/nominal_C_robodk.json`) muss nach dem Zusammenführen in main dort neu erzeugt
  werden (`py.exe robodk/simulate.py --animate`), weil der Prüfstempel aus dem Worktree keinen Git-Commit enthält.

## Neu erzeugen

```
py.exe tools/make_job.py
py.exe tools/make_stone_list.py
py.exe tools/make_guides.py
py.exe tools/plan_layout.py
py.exe robodk/simulate.py --animate --video results/l_wall_sim.mp4
```
