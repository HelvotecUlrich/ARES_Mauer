# Testplan Realtest ARES Mauer

| | |
|---|---|
| Dokument | Testplan für die ersten Realtests des Mauerroboters (UR5 CB3 auf ARES) mit der Mauer-HMI |
| Stand | 2026-10-08, Entwurf. Samuel gibt ihn vor T0 frei |
| Software-Basis | `main` 7ab0be8 (Planung, Motion Guard, RoboDK-gestempelte Jobs, PolyScope-3.3-IK-Prüfung 8894751, Preflight-Blocker für PLACEHOLDER 7d9459f, Konfig-Hash ohne `[hmi*]` fce214f) und Branch `hmi-integration` a443ba5 (Mauer-HMI mit den Review-Korrekturen vom 2026-10-08, 7ca642c–a443ba5; noch **nicht** in `main`, siehe S1) |
| Konfiguration | `config/station.toml`, Hauptkonfiguration "C A 5½ → B → C" (ohne Variante). Die Variante `c_acb` wird hier nicht getestet |
| Job | `data/jobs/nominal_C_robodk.json` (RoboDK-verifiziert). Er wird nach dem Konfigurations-Freeze neu erzeugt (Abschnitt 3.5) |
| Quellen | README.md, docs/ARCHITECTURE.md, docs/CAMERA_SETUP.md, hmi/README.md, docs/HMI_DESIGN.md §14 und §17, Docstrings in tools/*.py, `mauer/sequencer.py` `preflight_real`, `mauer/ares/ads.py`, targets/guides/README.md, targets/guides/floor_plan.md |

**Hinweise zu den Zahlen.** Grenzwerte und Parameter stammen aus `config/station.toml` (mit Statuskennung CONFIRMED / ASSUMPTION / PLACEHOLDER) oder aus den genannten Werkzeugen. Schlägt dieses Dokument selbst einen Wert vor, steht dort **Vorschlag**. Die Job-Daten in Anhang A (Stops, Magazinplätze, Steinfolge, Fahrwege) und die Bestätigungstexte gelten für den Job vom 2026-10-08 02:53 und sind gegen dessen Dry-run und einen SIM-Lauf mit `main` 7ab0be8 geprüft. Nach dem Neuaufbau (3.5) liest man sie aus dem neuen Dry-run und der neuen Steinliste neu ab. Ein Ergebnis nennt immer die PLACEHOLDER, von denen es abhängt (CLAUDE.md).

---

## 1 Ziel und Ablauf

Die Tests gehen vom Einfachen zum Komplexen. Ein Test beginnt erst, wenn der vorige bestanden ist.

- **Phase 1 – Messen und Kalibrieren (T0–T5a).** Läuft mit dem Software-Stand nach dem Merge der HMI (S1). In der HMI bleibt **Start** im REAL-Modus gesperrt; bekannte Preflight-Blocker siehe T1.
- **Freeze (3.5).** Die Messwerte kommen in die Konfiguration. Dann folgen Commit, RoboDK-Verifikation (neuer Stempel), Dry-run und Steinliste.
- **Phase 2 – Bauen (T5b–T10).** Läuft nur mit dem gestempelten Job und mit "REAL preflight ok".

| Test | Inhalt | Was bewegt sich | Gate |
|---|---|---|---|
| T0 | Netzwerk, IPs, Sicherheitsbegehung, Not-Halt-Test | nichts | – |
| T1 | HMI: SIM-Probe, REAL verbinden, Preflight | nichts (nur Bremsen lösen am UR) | T0 |
| T2 | ADS, kleine Relativfahrten (erst aufgebockt, dann am Boden) | ARES | T1 |
| T3 | IK-Prüfung PolyScope 3.3, UR-Montage auf ARES, Magazin, Massen, Greifer, Parkpose, Kamera-Montagecheck | UR (Pendant, kurze Werkzeugfahrten) | T2 |
| T4 | Hand-Auge-Kalibrierung auf ARES | UR (orbit) | T3 |
| T5a | Bodenaufbau, W0/W1 von Stop 0, Einschwingzeit | ARES (manuell), UR (Freedrive) | T4 |
| Freeze | Werte eintragen, Commit, RoboDK-Stempel, Dry-run, Steinliste | – | T5a, S1, S2 |
| T5b | Wandrahmen-Fit durch den Sequencer, erste ARES-Fahrt aus dem HMI-Prozess | UR, ARES (≈ 30 mm) | Freeze |
| T6 | ein Stein Magazin → Wand im Schrittmodus | UR | T5b |
| T7 | Stop 0, Teil 1: 14 Steine, bis das Magazin leer ist | UR | T6 |
| T8 | Stationsfahrt (Reload 1) und Rest von Stop 0 | ARES (≈ 1,3 m, 2 × 90°), UR | T7 |
| T9 | Beinwechsel A → B, erste 6 Steine von B an der Ecke | ARES, UR | T8 |
| T10 | ganze Mauer: a) Fortsetzung bis zum Ende, b) Gesamtlauf aus leerem Zustand | alles | T9 |

**Warum T7 keinen ganzen Stop umfasst.** Stop 0 hat 24 Steine, das Magazin startet mit 14. Nach dem 14. Stein muss ARES zur Station fahren. T7 endet vor dieser Fahrt, T8 fährt zur Station und setzt die restlichen 10 Steine. Mit dem Ende von T8 ist Stop 0 vollständig gebaut.

**T5b bis T9 sind ein einziger HMI-Lauf** (Prepare einmal mit `stops 0 to 3`). Jeder Testabschnitt endet mit **Decline** an einer Bestätigung bei leeren Backen. Dann steht der Lauf auf "aborted", ist aber wiederaufnehmbar, und der nächste Test beginnt mit **Resume**. Die HMI bleibt dazwischen geöffnet, denn der Laufzustand liegt nur im Speicher.

**Zählweise.** Stops werden überall ab 0 gezählt (HMI-Anzeige "stop 0 (0-3)", Run-Log, Stops-Felder, Planansicht); die Teilfahrten einer Route ab 1 ("leg 1/2").

---

## 2 Sicherheit und Rollen

### 2.1 Rollen (Festlegung für den Test)

| Rolle | Aufgabe |
|---|---|
| Bediener | Laptop, HMI, Bestätigungen; die Hand ist an HALT (Leertaste) |
| Sicherheitsperson | erreicht beide Not-Halte (ARES und UR-Pendant), sieht ARES und den Arm, ruft "Stopp" |
| Protokoll | Ergebnistabelle, Fotos, Videos; der Bediener kann das übernehmen |

Es sind immer mindestens zwei Personen anwesend. Auf "Stopp" drückt der Bediener sofort HALT, die Sicherheitsperson bei Gefahr den Not-Halt.

### 2.2 Stop-Funktionen

| Funktion | ARES | UR | Lauf |
|---|---|---|---|
| **Not-Halt ARES** (Hardware, Sicherheitskreis mit Scanner) | SAFETY STOP | Wirkung unbekannt, wird in T0 geprüft | Sequencer meldet Fehler ("error") |
| **Not-Halt UR** (Pendant) | Wirkung unbekannt, wird in T0 geprüft | Not-Halt | Sequencer meldet Fehler |
| **HALT** in der HMI (Knopf, Leertaste oder Esc, auf jedem Tab) | ein Schreibzugriff `bCmdMoveAbort` + alle Jog-Bits FALSE, Rampe; Antriebe bleiben an, MANUAL bleibt. REAL: die ADS-Verbindung des Sequencers wird gesperrt (keine neue Startflanke bis Start / Resume) | REAL: der URLink wird gesperrt (kein neues Programm außer dem Abbruch bis Start / Resume), dann `URLink.abort()` (stopl + Dashboard-Stop) | "aborted", wenn die Bewegung noch nicht gesendet war; "error", wenn eine laufende Bewegung gestoppt wurde. `halt` (und REAL `halt_result`) im Run-Log. Resume läuft im Schrittmodus |
| **Decline** (Bestätigungsleiste) | – | – | Bewegung wird nicht ausgeführt, sofort "aborted"; geht auch mit Stein im Greifer (dann vor dem Resume **Jaws empty**) |
| **Pause / Abort** (Mauer-Tab) | – | – | wirkt an der nächsten Bewegungsgrenze mit leeren Backen: eine laufende Ablage läuft noch zu Ende |
| **Abort move** (ARES-control-Tab) | bricht die manuelle Relativfahrt ab | – | – |
| **Ctrl-C** in `calib_handeye.py` / `measure_target.py` | – | bricht den laufenden UR-Block ab | – |
| HMI schließen | Heartbeat endet zuerst: PLC bricht eine Fahrt nach 500 ms ab, MANUAL fällt nach 2 s | ein laufender UR-Block fährt zu Ende | Schließen wird verweigert, solange ein Lauf aktiv ist oder eine Relativfahrt läuft (`bMoveActive`) |

HALT ist eine Bedienfunktion. Die Sicherheitsfunktion sind allein die Not-Halte.

**Leertaste und Esc wirken nur, solange die HMI das aktive Fenster ist** (oder einer ihrer Dialoge). Ist mit geöffnetem REAL-Rig ein anderes Fenster vorne (RoboDK, Explorer, Editor), zeigt die HMI über den Tabs ein rotes Banner "Keyboard HALT (Space / Esc) inactive". Der HALT-Knopf wirkt immer.

### 2.3 Regeln

1. **Nur ein ADS-HMI:** die Mauer-HMI mit `--ares`, nie zusätzlich die MA-HMI `amr_hmi`. Die TwinCAT-Laufzeit des C6030 ist gestoppt (sie startet von selbst, sonst `bExtActive`).
2. **Ein Programm pro Gerät:** Kamera und UR werden immer nur von einem Werkzeug gleichzeitig benutzt. Vor CLI-Werkzeugen (`cam_check`, `ur_check`, `measure_target`, `calib_handeye`) in der HMI **Release** drücken oder die HMI schließen.
3. **Schrittmodus** gilt für jede erstmalige Aktion. **Go** wird erst 0,5 s nach Erscheinen der Bestätigung aktiv ([hmi] confirm_arm_s). Vor jedem Go auf Roboter und ARES schauen, nicht auf den Bildschirm.
4. Den **Arbeitsraum** betritt niemand, solange eine Bewegung läuft oder eine Bestätigung offen ist. Betreten nur bei gestopptem Lauf ("aborted" oder "paused"); für längere Arbeiten zusätzlich den UR-Not-Halt drücken.
5. Die **Leertaste löst HALT aus.** Mit ihr keine Knöpfe betätigen.
6. **Die HMI bleibt das aktive Fenster**, solange ein Lauf aktiv ist. Kein Klick in RoboDK, Explorer oder andere Programme; zeigt die HMI das rote Banner, zuerst in die HMI klicken. Während eines Laufs sperrt die HMI ihre eigenen Knöpfe, die Fenster öffnen (Open log folder, Open snapshot folder, Open summary, Twin einschalten / Restart / Save view PNG).
7. **Manuelle ARES-Fahrten** (Jog, Relativfahrt im ARES-control-Tab, `ares_check.py`) haben keine Verriegelung zum UR. Vorher steht der UR in der Parkpose (ab T3) oder in einer kompakten Pose, und kein Programm läuft. Der Sequencer verriegelt nur seine eigenen Fahrten. **Start** und **Resume** sind gesperrt, solange ARES fährt (`bMoveActive` / `bAmrMoving`).
8. Die **ersten UR-Bewegungen** laufen mit niedrigem Speed-Slider am Pendant (so verlangt es die Bestätigung von `calib_handeye.py`). Die [ur]-Geschwindigkeiten bleiben auf den vorsichtigen Erstwerten (ASSUMPTION).
9. Die **Räder** fahren nie über Leitstreifen oder Board-Platten (4 mm MDF): Im Routenmodell sind sie Hindernisse.
10. **Keine Finger zwischen die Backen.** Der EHPS-20-A greift mit 218 N gesamt (Festo-Datenblatt laut input/kamerahalter_handoff.md).
11. **Ab T4** wird nicht mehr an Kamera, Adapter, Fokus oder Blende gearbeitet. Sonst sind Intrinsik und Hand-Auge-Kalibrierung neu fällig.
12. **In Phase 2** keine Änderung unter `mauer/`, `robodk/`, `tools/make_job.py`, `cad/` oder `config/` (außer den `[hmi*]`-Tabellen) ohne neuen RoboDK-Stempel (3.5).

### 2.4 Allgemeine Abbruchkriterien

Bei jedem dieser Punkte wird der Test beendet und die Ursache geklärt, bevor es weitergeht:

- unerwartete Bewegungsrichtung oder unerwarteter Weg; jeder unbeabsichtigte Kontakt von Arm, Stein oder ARES;
- Ablehnung durch den Motion Guard, ein UR-Protective-Stop oder einer der PLC-Abbruchcodes 23, 24, 28, 29 ohne Erklärung;
- ein Stein fällt, kippt, sitzt nicht, oder ein Nachbarstein wurde verschoben;
- ARES neigt sich sichtbar oder ein Rad hebt ab. Die Kippsicherheit ist nur grob statisch abgeschätzt (README), mit realen Massen noch nicht;
- Warnungen zu Heartbeat oder Verbindung (PLC HB, RTDE-Alter > 0,5 s, "RTDE lost during the block");
- wiederholt Fit-RMS > 1,0 mm oder Sprungfehler;
- Geruch, Geräusch oder Wärme an Antrieben oder Greifer.

### 2.5 Offene Sicherheitspunkte (in T0 klären und dokumentieren)

- Sind die Not-Halte von ARES und UR gekoppelt? Unbekannt (CAMERA_SETUP §10).
- Schutzfelder des ARES-Frontscanners bei MANUAL-Relativfahrten: An den Stops ist die Wand 220 mm entfernt, am Dock liegt die ARES-Front 70 mm vor der Station ([pickup_station] ares_xyz).
- In der PLC gibt es keine Verriegelung zwischen UR und ARES. Nur der Sequencer verriegelt: ARES fährt nur bei geparktem, ruhendem Arm.
- Bedienmuster A in **einem** Prozess (HMI-ADS-Worker plus `AresAds` des Sequencers) ist nur gegen die Fake-PLC getestet. Der erste reale Einsatz ist T5b, mit einer kleinen Korrekturfahrt.

---

## 3 Voraussetzungen

### 3.1 System und Netzwerk

| Gerät | Adresse | Ports / Kennung | Quelle |
|---|---|---|---|
| Laptop, NIC "Ethernet" | 192.168.1.20 (ARES), 192.168.56.20 (UR), 192.168.50.1/24 (Kamera) | AMS-NetID 192.168.1.20.1.1 (TwinCAT-Route zur CX9240) | [camera] ip-Kommentar, CAMERA_SETUP §9 |
| ARES CX9240 (PLC ≥ v2.1, Interface v2) | 192.168.1.10 | AMS 192.168.1.10.1.1, Port 851, ADS-TCP 48898 | [ares_ads] CONFIRMED |
| UR5 CB3 (PolyScope 3.3.3.292) | 192.168.56.101 | Dashboard 29999, URScript 30002, RTDE 30004 | [ur] CONFIRMED |
| IDS GV-51F0CP-M-GL | 192.168.50.10/24, Gateway 192.168.50.1 | Seriennummer 4110073444, Firmware ≥ 3.31 | [camera] CONFIRMED |
| C6030 (ARES-IPC) | – | TwinCAT-Laufzeit **gestoppt** | ads.py Preflight-Text |
| RoboDK (nur Twin, optional) | lokal | eigene Instanz, API-Port ≥ 20630 ([hmi.twin]) | hmi/README.md |

Versorgung: Kamera über den UR-Werkzeugstecker mit 24 V ([camera] power, ASSUMPTION; Kamera ≈ 0,18 A, Stecker max. 600 mA). Greifer EHPS-20-A (bis 2 A) über ARES-24 V, vom UR kommen nur DO0 (öffnen) und DO1 (schließen).

### 3.2 Software (Blocker für Phase 2)

| Nr. | Punkt | Spätestens vor |
|---|---|---|
| S1 | `hmi-integration` (mit den Review-Korrekturen) in `main` mergen, mit allem, was `main` seitdem bekommen hat (21b7731 bis 7ab0be8; Plan D6). Dabei: in `hmi/core/rigs.py` `MotionGuard(..., approach_mm=job.approach_mm)`; `SimRig` mit Motion Guard; HMI-Start bei Stop k > 0 wie `tools/run_job.py` (Magazinbelegung `restart_fill`, `declare_placed`, Abfrage "Backen leer"); die pauschale 3.3-Sperre `IK_GUARD_33` in `hmi/core/preflight.py` durch die Prüfung von `[ur] ik_check` ersetzen (C4) | T1 |
| S2 | PolyScope-3.3-Variante von `ik_guard`: **erledigt** in `main` 8894751 (`[ur] ik_check = "get_inverse_kin"`). Offen ist nur die Prüfung am Roboter, dass das `qnear`-Schlüsselwort auf 3.3 funktioniert (T3 Schritt 0) | T5b |
| S3 | Tests grün: `py.exe -m pytest -q tests` | Freeze |
| S4 | Freeze mit RoboDK-Stempel (3.5) | T5b |
| S5 | SIM-Probe auf dem Testlaptop bestanden (T1a) | T1b |
| S6 | Empfohlen: Die HMI zeigt vor einem REAL-Start die Magazinbelegung und fragt "Backen leer / Magazin wie Liste / Station voll" ab (Anhang C1). Bis dahin gilt die manuelle Checkliste in T5b | T5b |

### 3.3 Material und Aufbau

- **Bodenaufbau** nach targets/guides/README.md: Lasertest (`laser_test.dxf`, zwei Kegel) bestanden, Blätter 1–4 geschnitten, 52 Kegel und Ersatz gedruckt, Teppichband. Die Board-Platten W0–W6 liegen auf ihren V-Laschen, W7 ist Reserve, S0/S1 sitzen in den Stationsfenstern. Zuordnung der Boards: Anhang A.4 (= targets/README.md §4, seit c10d72a für das C).
- **Magazinhalter** auf dem Deck an den Positionen aus [deck] (Anhang A.5), mit Zentrierkegeln.
- **Steine:** 64 Vollsteine und 12 Halbsteine für die Mauer. Dazu kommen die Stationsfüllungen laut Steinliste: Die Station wird bei jedem Nachfüllen ganz gefüllt, man braucht also mehr als 76. Reserve bereitlegen. Für T3: je 3 Voll- und Halbsteine zum Wiegen und ein Halbstein zum Vermessen.
- **calib-Board** auf seiner Platte, Klebeband zum starren Befestigen.
- **Aufbockmaterial** für ARES (alle Räder frei, gegen Abrutschen gesichert).
- **Messmittel:** Maßband ≥ 3 m, Stahlmaß, Messschieber, Fühlerlehren (0,05–1,5 mm), Winkel, Wasserwaage oder Neigungsmesser, Waage für Steine, Klebeband zum Markieren, Kamera/Handy (Video, Zeitlupe).
- ARES-Akku geladen (Battery-Tab). Licht im Labor gleichmäßig, kein direktes Sonnenlicht auf den Boards.

### 3.4 Offene Parameter: was in welchem Test gemessen oder bestätigt wird

Alle Werte stammen aus `config/station.toml` (main 7ab0be8). Jeden Messwert mit Status, Datum, Test und Methode eintragen (Tabelle 6.4). "Preflight-Blocker" heißt: `preflight_real` lehnt den REAL-Lauf ab, solange der Wert PLACEHOLDER / UNKNOWN ist (7d9459f bzw. [half_brick]).

| Schlüssel | Heute | Maßnahme | Test | Spätestens vor | Wirkung bei Fehler |
|---|---|---|---|---|---|
| [ur5] mount_x | 353.625, CONFIRMED ("exact position follows") | nachmessen | T3 | Freeze | Magazin-Pick, Motion Guard |
| [ur5] mount_y | 0.0, ASSUMPTION | messen | T3 | Freeze | Magazin-Pick |
| [ur5] mount_z | 333.6, **PLACEHOLDER** (Deck, ohne Adapterplatte) | messen; Gegenprobe in T5a | T3 | Freeze (Preflight-Blocker) | Pick-Höhe 1:1, also Ablagehöhe |
| [ur5] mount_rz | 0.0, **PLACEHOLDER** | messen (Pendant +X) | T3 | Freeze (Preflight-Blocker) | 1° versetzt Magazinreihe 0 (653,6 mm hinter der UR-Achse) um ≈ 11 mm |
| [tool] tcp_z | 147, ASSUMPTION | prüfen: Backen fassen den Stein genug, Greiferkörper frei von den Zapfen | T3 | Freeze | Greiftiefe, Kollision |
| [brick] mass_kg | 3.0, ASSUMPTION ("about 3 kg") | wiegen | T3 | Freeze | Nutzlast: 1,68 + Stein ≤ 5,0 kg |
| [half_brick] length / width / height / pin_across / pin_length | **PLACEHOLDER** | realen Halbstein vermessen | T3 | Freeze (Preflight-Blocker) | Ablage der Halbsteine |
| [half_brick] mass_kg | 0.0, UNKNOWN | wiegen | T3 | Freeze (Preflight-Blocker) | Nutzlast |
| [ur] payload_cog_mm | [0, 0, 60], **PLACEHOLDER** | bestimmen | T3 | Freeze (Preflight-Blocker) | Nutzlastmodell des UR (Kollisionserkennung) |
| [ur] park_q_deg | ASSUMPTION ("verify slowly on the robot") | langsam am Roboter prüfen | T3 | T5b | Kollision in der Parkpose |
| [ur] grip_wait_s | 0.5, ASSUMPTION | Schließzeit auf einen Stein messen | T3 | Freeze | Stein nicht gegriffen |
| [ur] ik_check | "get_inverse_kin", CONFIRMED; ASSUMPTION: `qnear` wirkt auf 3.3 | `calib_handeye.py plan --check` (ohne Bewegung) | T3 | T5b | jede Blickpose, jeder Pick und jedes Place scheitert |
| [ur] reg_started / reg_done / reg_error (20–22) | ASSUMPTION "frei" | klären, ob die PLC oder ein Feldbus sie nutzt | T0 | T5b | Blockerkennung gestört |
| [deck] holder_z | 20, **PLACEHOLDER** | messen | T3 | Freeze (Preflight-Blocker) | Pick-Höhe 1:1 |
| [deck] magazine_rows_dx / magazine_y / half_positions / magazine_layers | ASSUMPTION | Lage der Halter zur UR-Basis messen | T3 | Freeze | Pick-Lage, damit Ablage längs (außerhalb der Kameraregelung) |
| [deck] layers | 2, PLACEHOLDER | keine (nur Raster der Reichweitenstudie) | – | – | – |
| [ares] deck_top_z | 333.6, CONFIRMED (STEP) | Deckhöhe am UR messen (gefedert) | T3 | – | Höhen |
| [camera] exposure_us | 60000, CONFIRMED (Licht am Tisch) | unter Testlicht prüfen | T4 | Freeze | Erkennung |
| [camera] gain | 1.0, PLACEHOLDER | prüfen und bestätigen | T4 | Freeze | Erkennung |
| [camera] settle_s | 2.0, **PLACEHOLDER** | mit Boden-Board messen (ARES schwingt) | T5a | Freeze (Preflight-Blocker) | Messfehler durch Schwingen |
| [camera.mount] xyz / rpy | Designwert, ASSUMPTION | Orientierung prüfen; die Lage liefert `calib/handeye.json` | T3, T4 | – | Laufzeit nutzt `calib/handeye.json` |
| [boards.calib] marker_mm | 11, PLACEHOLDER | messen | T4 | Freeze | gering |
| [boards.calib] xyz / rpy | PLACEHOLDER | keine (orbit braucht sie nicht) | – | – | – |
| [boards.ref] square_mm / marker_mm | 16 / 12, **PLACEHOLDER** | Druck messen | T5a | Freeze (square_mm: Preflight-Blocker) | Maßstab: 0,1 % ergeben 0,32 mm Tiefe bei 320 mm |
| [[targets]] W0–W6 xyz / rpy | ohne eigenes Tag, folgen aus den PLACEHOLDER-Maßen in [plates] | per Bodenprüfung bestätigen | T5a | Freeze | Wandlage 1:1 |
| [[targets]] S0/S1 xyz | ASSUMPTION | Station nach Plan legen und prüfen | T5a | Freeze | Dock-Messung |
| [plates] block_width / block_length | 120 / 200, PLACEHOLDER | Streifenbreite und V-Laschen-Teilung messen | T5a | Freeze | Board-Lage |
| [wall] base_z | 4.0, ASSUMPTION | MDF-Dicke messen | T5a | Freeze | Höhe von Lage 0 |
| [wall] corner_gap_mm | 1.0, ASSUMPTION | an den Eck-L-Stücken prüfen, in T9 messen | T5a, T9 | Freeze | Eckfuge |
| [[wall.legs]] xyz_in_wall | ASSUMPTION | Diagonalen nach floor_plan.md | T5a | Freeze | Lage der Schenkel B und C |
| [guides] socket_clearance, peg_hole_d, vtab, dovetail, kerf_mm | ASSUMPTION | Lasertest (guides-README Schritt 1) | vor T5a | – | Passungen |
| [pickup_station] (alle) | ASSUMPTION | nach Plan legen; Dock in T8 | T5a, T8 | Freeze | Dock, Stationsgriffe |
| [routes], [sequencer] (alle) | ASSUMPTION (Erstwerte) | nicht messen; beobachten. Anpassen nur gesammelt, mit neuem Stempel | T5b–T10 | – | – |

### 3.5 Freeze und Job-Neuaufbau (zwischen T5a und T5b)

1. S1–S3 sind erledigt und in `main`.
2. Alle Messwerte aus T0–T5a in `config/station.toml` eintragen, mit Status CONFIRMED und Quelle, z. B. `mass_kg = 3.12  # CONFIRMED 2026-10-xx (Realtest T3, Waage, max. von 3 Steinen)`. Alle [half_brick]-Schlüssel und die sechs Preflight-Blocker aus 3.4 ersetzen, sonst blockiert der Preflight.
3. Wenn sich Werte geändert haben, die den Bodenaufbau bestimmen ([plates], [[targets]], [guides], [[wall.legs]], [pickup_station]): `py.exe tools/make_guides.py` und die Abweichung zum verlegten Aufbau klären.
4. Committen. Für den Stempel muss der Baum unter `mauer/`, `robodk/`, `tools/make_job.py`, `cad/` und `config/` sauber sein.
5. RoboDK-Verifikation: `py.exe robodk/simulate.py --animate`. Sie läuft in einer eigenen RoboDK-Instanz und dauert etwa 25–35 min. Ändern sich [ur5] oder [tool] (oder die Steinmaße, [wall] base_z / courses), berechnet `simulate.py` zuerst die Reichweitentabellen neu; das dauert deutlich länger, und Stops und Steinzahlen können sich ändern. Ergebnis ist `data/jobs/nominal_C_robodk.json`. Gestempelt wird nur ein vollständiger, kollisionsfreier Lauf, und nur wenn sich der Code während des Laufs nicht geändert hat (87757fa); sonst löscht das Werkzeug eine ältere gestempelte Datei.
6. `py.exe tools/run_job.py data/jobs/nominal_C_robodk.json --dry-run` ausdrucken und Anhang A damit abgleichen. Steinliste neu erzeugen: `py.exe tools/make_stone_list.py --job data/jobs/nominal_C_robodk.json`; sie enthält die Startfüllung von Magazin und Station und die Nachfüllungen.
7. In der HMI den Job laden, REAL wählen, **Prepare / Connect**: Die Anzeige lautet "REAL preflight ok".

**Regel:** Jede spätere Änderung unter den Pfaden aus Schritt 4 macht den Stempel ungültig (Preflight: "the planning code / CAD changed since the RoboDK verification" bzw. "the config (config/station.toml) changed since the RoboDK verification"). Dann die Schritte 4–7 wiederholen. Ausgenommen sind die `[hmi*]`-Tabellen der Konfiguration (fce214f) und `calib/`: eine neue Hand-Auge-Kalibrierung braucht keinen neuen Stempel.

**Konfiguration nach dem Laden:** Die HMI benutzt die Konfiguration, die beim Laden des Jobs gelesen wurde. Wird `config/station.toml` danach geändert, meldet der Preflight "changed on disk since the job was loaded"; dann **Release** und den Job neu laden.

**Datensicherung:** `data/` ist nicht versioniert. Nach jedem Testtag `data/runs/`, `data/he_*`, `data/mount_check/` und `data/T5/` sichern.

---

## 4 Tests

Alle Befehle laufen im Repo-Wurzelverzeichnis `C:\Users\samue\ARES_Mauer` mit `py.exe` (aus cmd, PowerShell oder WSL). Konsolenausgaben speichert man mit `| Tee-Object <Datei>` (PowerShell) oder `| tee <Datei>` (WSL).

### T0 Netzwerk, IPs und Sicherheitsbegehung

**Zweck:** Alle Geräte sind unter den erwarteten Adressen erreichbar, ohne dass sich etwas bewegt. Alle Beteiligten kennen die Stop-Funktionen. Das Zusammenspiel der Not-Halte ist bekannt.

**Voraussetzungen:** ARES eingeschaltet, PLC in RUN, C6030-Laufzeit gestoppt. UR-Steuerung an, Arm "Power on" (die Bremsen dürfen zu bleiben), damit die Werkzeugspannung die Kamera versorgt. Der Laptop hängt am ARES-Netz. Keine HMI läuft.

**Ablauf:**
1. Software-Stand: `git log -1 --oneline` und `git status` (sauber) notieren.
2. `ipconfig`: Die Laptop-NIC hat die Adressen aus 3.1.
3. `ping 192.168.1.10`, `ping 192.168.56.101`, `ping 192.168.50.10`.
4. Kamera: `py.exe tools/cam_check.py list` (Seriennummer 4110073444 "openable", Firmware ≥ 3.31), dann `py.exe tools/cam_check.py grab -n 5` (am 2026-10-06: 5/5 Bilder, 123 ms).
5. UR ohne Bewegung: `py.exe tools/ur_check.py info` zeigt PolyScope-Version, Robot-/Safety-Mode, Gelenkwinkel und Werkzeugspannung.
6. ARES, nur lesend: `py.exe tools/ares_check.py status` zeigt Interface ≥ 2, Build ≥ v2.1 und keine externe Steuerung. Ohne HMI meldet der Preflight "kein MANUAL" und "Heartbeat steht"; das ist erwartet.
7. Klären und notieren: Nutzt die PLC oder ein Feldbus die UR-Register 20–22? Welche Scannerfelder sind bei MANUAL-Relativfahrten aktiv (Abstände siehe 2.5)?
8. Begehung mit allen Beteiligten: Arbeitsraum abgrenzen. Der Bodenaufbau mit allen ARES-Lagen, Wegen und Drehkreisen misst 2939 × 1812 mm (floor_plan.md). Dazu kommt Freiraum für den geparkten Arm: Sein TCP steht 704 mm vor base_link ([ur] park_q_deg). Not-Halt-Taster zeigen, Abschnitt 2.2 und 2.3 durchgehen, das Rufzeichen "Stopp" vereinbaren.
9. Not-Halt-Test ohne Bewegung:
   - a) ARES-Not-Halt drücken. `ares_check.py status` zeigt den AMR-Zustand 2 (SAFETY STOP). Mit `ur_check.py info` prüfen, ob der UR reagiert.
   - b) UR-Not-Halt am Pendant drücken und den ARES-Zustand prüfen.
   - Beide lösen: den UR am Pendant, ARES über die Startsequenz in T1.

**Erwartetes Ergebnis:** Alle Geräte antworten, die Kamera lässt sich öffnen, die Versionen stimmen. Beide Not-Halte wirken.

**Bestanden, wenn:** Punkte 2–6 ohne Fehler; das Not-Halt-Verhalten ist in Tabelle 6.2 eingetragen; die Antworten zu Punkt 7 liegen vor; alle Rollen sind besetzt.

**Protokollieren:** Ausgaben der Punkte 4–6, Antworten zu Punkt 7, Tabelle 6.2.

**Sicher abbrechen:** Es bewegt sich nichts. Bei unerwartetem Verhalten nicht weitermachen.

---

### T1 HMI-Start und Preflight

**Zweck:** Die Mauer-HMI läuft auf dem Testlaptop und verbindet sich im REAL-Modus mit allen drei Teilsystemen. Sie zeigt einen vollständigen Preflight, dessen Punkte alle erklärbar sind. Die Bedienung ist vorher in SIM geübt.

**Voraussetzungen:** T0 bestanden; S1 erledigt. Der UR ist eingeschaltet und die Bremsen sind gelöst, am Pendant oder mit `py.exe tools/ur_check.py ready` (die Gelenke zucken dabei kurz). Kein UR-Programm läuft, `amr_hmi` ist geschlossen.

**Ablauf T1a – SIM-Probe** (ohne Hardware, auch am Vortag möglich):
1. `py.exe -m hmi --job data/jobs/nominal_C_robodk.json --twin`. Ohne `--ares` zeigt der Titel "ADS off", REAL ist gesperrt.
2. Mauer-Tab: SIM, Szenario `realistic`, `stops 0 to 0`, Schrittmodus an, **Prepare / Connect**, dann **Start**.
3. Einige Bewegungen mit **Go** bestätigen. Bei einer Bestätigung mit Stein im Greifer **Pause** drücken: Die Ablage läuft noch und wird noch bestätigt, dann wechselt der Zustand auf "paused". **Resume**.
4. Mit Stein im Greifer auf dem Camera-Tab die Leertaste (HALT) drücken. Resume wird verweigert. **Jaws empty**, dann **Resume** bis "done".
5. Twin-Tab ("running"), Camera-, Wall-pose- und Run-log-Tab ansehen. Im Run-Log steht der HALT als Eintrag `halt`.
6. **Release** und Fenster schließen. Die RoboDK-Instanz des Twins schließt sich mit.

**Ablauf T1b – REAL verbinden:**
1. `py.exe -m hmi --ares --job data/jobs/nominal_C_robodk.json` (unter Windows auch `hmi.cmd --ares --job ...`). Der Titel lautet "Mauer HMI - … - config main - …", ohne "ADS off".
2. ARES-control-Tab: Statusleiste "Connected", PLC-Build, Interface v2, PLC HB läuft. Startsequenz **Safety Run** → **AMR Reset** → **Start** → **Manual**, bis Zustand 7 MANUAL MODE. Diagnostics ohne Fehler, Akkustand im Battery-Tab notieren.
3. Mauer-Tab: Der Job und "config main" sind sichtbar. REAL, `stops 0 to 0`, Schrittmodus an, camera loop an, save images an, dann **Prepare / Connect**. Das öffnet den UR-Link, die eigene ADS-Verbindung des Sequencers, die IDS-Kamera und die Kalibrierdateien.
4. Die Preflight-Liste lesen. **Start** bleibt gesperrt, solange ein Punkt blockiert; in Phase 1 ist das erwartet. Bekannte Blocker vor dem Freeze (Stand `main` 7ab0be8), alle mit der Quelle job/config:
   - `[ur5] mount_z PLACEHOLDER`, `[ur5] mount_rz PLACEHOLDER`, `[deck] holder_z PLACEHOLDER`, `[ur] payload_cog_mm PLACEHOLDER`, `[boards.ref] square_mm PLACEHOLDER`, `[camera] settle_s PLACEHOLDER` (je "- measure it (no camera correction: …)");
   - `[half_brick] mass_kg <= 0 (UNKNOWN)` und `[half_brick] height, length, pin_across, pin_length, width PLACEHOLDER`;
   - `the planning code / CAD changed since the RoboDK verification (7a98fab5)` und `the config (config/station.toml) changed since the RoboDK verification (7a98fab5)` (bis zum neuen Stempel, Plan D8 bzw. Freeze);
   - `config/station.toml changed since the job was built (sha256 differs) - rebuild the job` (bis zum neuen Stempel);
   - Quelle UR: keiner, wenn S1 den pauschalen Blocker `IK_GUARD_33` durch die Prüfung von `[ur] ik_check` ersetzt hat; sonst `PolyScope 3.3: get_inverse_kin_has_solution missing …`.
   - Punkte der Quellen HMI, ARES oder camera dürfen nicht auftreten.
5. UR-Tab: RTDE-Alter < 0,5 s, Controller 3.3, Robot mode RUNNING, Safety NORMAL, Gelenkwinkel wie am Pendant. Camera-Tab: **Grab** liefert ein Bild; **Live** (1 Hz) ein- und wieder ausschalten. Statusleiste über den Tabs prüfen.
6. HALT ohne Bewegung: Leertaste. Die Statuszeile zeigt "HALT sent", ARES bleibt in MANUAL. Die HMI zeigt "HALTED"; URLink und die ADS-Verbindung des Sequencers sind gesperrt, bis **Start** (bzw. später **Resume**) gedrückt wird. Im Run-Log stehen `halt` und `halt_result` (was der UR-Abbruch gemacht hat).
7. Tastatur-HALT-Banner: ein anderes Fenster anklicken (z. B. die Konsole). Über den Tabs erscheint rot "Keyboard HALT (Space / Esc) inactive". Zurück in die HMI klicken: Das Banner verschwindet.
8. Optional Twin: HMI mit `--twin` starten. Der Twin-Tab zeigt "running" bei etwa 10 Hz ([hmi.twin] rate_hz), und PLC HB bleibt stabil.
9. **Release**, dann die HMI schließen. ARES verlässt nach etwa 2 s MANUAL; mit `ares_check.py status` prüfen.

**Bestanden, wenn:** Die SIM-Probe ist vollständig (Pause, HALT, Jaws empty, Resume, "done"). In REAL sind alle Teile verbunden (kein "not connected" oder "not open"), der Preflight enthält nur die bekannten Blocker, die Anzeigen für UR, Kamera und ARES sind plausibel. HALT hat bei stehenden Systemen keine Wirkung außer der Sperre. Das Banner erscheint und verschwindet. Release und Schließen laufen sauber.

**Protokollieren:** Screenshots von Preflight-Liste, UR-Tab und ARES-control-Tab; Run-Log-Ordner der SIM-Probe; Bemerkungen zur Bedienung.

**Sicher abbrechen:** Außer beim Bremsenlösen bewegt sich nichts. Bei Verbindungsfehlern **Release**, HMI schließen und T0 wiederholen.

---

### T2 ADS und eine kleine Relativfahrt

**Zweck:** Bedienmuster A funktioniert mit der Mauer-HMI: Die HMI besitzt Heartbeat, MANUAL und HALT, der Client `AresAds` schreibt nur die Fahrfelder. Richtung (+y = links) und HALT stimmen. Die reale Genauigkeit kleiner Fahrten ist bekannt.

**Voraussetzungen:** T1 bestanden. Der UR steht in einer kompakten Pose innerhalb der ARES-Grundfläche, kein Programm läuft, am besten ist er ausgeschaltet. Für T2a ist ARES aufgebockt (alle Räder frei, gesichert), für T2b steht er auf einer freien, ebenen Fläche mit mindestens 1 m Abstand ringsum.

**Ablauf T2a (aufgebockt):**
1. `py.exe -m hmi --ares`, dann die Startsequenz bis MANUAL. In der Odometrie-Karte **Reset pose**.
2. Im zweiten Terminal `py.exe tools/ares_check.py status`: Der Preflight meldet keine Probleme (Heartbeat ändert sich, MANUAL, keine externe Steuerung).
3. Relativfahrt-Panel: **Load test move** lädt "physisch links 100 mm bei 50 mm/s". **GO**, mit zweitem Klick innerhalb von 3 s. Erwartet: Ergebnis 1 "done, final error within tolerance", Odometrie +100 mm in y.
4. HALT-Test: vorwärts 500 mm bei 50 mm/s, **GO**, nach etwa 2 s die Leertaste. Erwartet: Ergebnis 20 "aborted: HALT / bCmdMoveAbort / bCmdStop", MANUAL bleibt.
5. Client `AresAds` in einem eigenen Prozess, wie in E003: `py.exe tools/ares_check.py move --dy 100 --yes` (Ergebnis "done"), zurück mit `--dy -100`.
6. HALT auf diesen Client: `py.exe tools/ares_check.py move --dx 500 --speed 50 --yes` und während der Fahrt HALT in der HMI. `ares_check` meldet ABORTED (20).
7. Optional Heartbeat-Verlust: während einer Fahrt aus Punkt 6 (ohne HALT) den HMI-Prozess hart beenden: das Konsolenfenster schließen, in dem `py.exe -m hmi` läuft, oder im Task-Manager den Prozess beenden. (Das HMI-Fenster selbst verweigert das Schließen, solange eine Relativfahrt läuft.) Die PLC bricht mit 26 "aborted: HMI heartbeat lost" ab und verlässt MANUAL nach etwa 2 s.

**Ablauf T2b (am Boden):**
1. Die Startlage an zwei Fahrzeugecken mit Klebeband markieren.
2. Relativfahrten mit 150 mm/s ([ares_ads] speed_mms, wie der Sequencer): vor 300 mm, zurück 300 mm, links 300 mm, rechts 300 mm. Nach jeder Fahrt die Verschiebung mit dem Maßband messen und mit der Odometrie vergleichen. Die Querfahrt (dy) wurde noch nie extern gemessen; E003 hat nur x gemessen.
3. Optional Drehung +90° und −90° mit 10 °/s ([ares_ads] rot_speed_degs). Um base_link einen Kreis von mindestens 0,8 m Radius freihalten (**Vorschlag**: Schätzung aus dem TCP der Parkpose bei 704 mm plus Greifer und Kamera).

**Bestanden, wenn:** Alle Fahrten enden mit Ergebnis 1, die HALT-Fahrten mit 20. +y fährt physisch nach links. HALT stoppt jede Fahrt mit Rampe. `ares_check` fährt nur, solange die HMI offen ist. In T2b ist die Abweichung jeder Fahrt dokumentiert. Referenz (README, E003, unbelastet): ±2–5 mm zwischen Stops. Deutlich größere Abweichungen vor T8 klären.

**Protokollieren:** PLC-Ergebniscodes und -texte, Tabelle 6.3 (Odometrie gegen Maßband), Ausgaben von `ares_check`.

**Sicher abbrechen:** HALT oder **Abort move**, bei Gefahr den Not-Halt. Hängt die HMI, bricht die PLC die Fahrt 500 ms nach dem Ende des Heartbeats ab.

---

### T3 IK-Prüfung, UR-Montage auf ARES, Nutzlast, Greifer, Parkpose

**Zweck:** Die IK-Prüfung für PolyScope 3.3 arbeitet auf dem Roboter. Lage des UR auf ARES, Lage des Magazins, Massen, Halbsteinmaße und Parkpose sind gemessen bzw. am Roboter geprüft. Die Kamera sitzt nach dem Umbau noch so, wie die Konfiguration sagt.

**Voraussetzungen:** T2 bestanden. ARES steht am Boden still, das Magazin ist leer, die Messmittel aus 3.3 liegen bereit, die HMI hat kein vorbereitetes Rig (Kamera und UR sind frei).

**Ablauf:**
0. **IK-Prüfung ohne Bewegung** (S2): `py.exe tools/calib_handeye.py plan --check`. Für jede geplante Pose läuft auf der Steuerung ein Block mit der IK-Prüfung aus `[ur] ik_check` ("get_inverse_kin", mit `qnear`), der sich nicht bewegt. Erwartet: je Pose "erreichbar" oder "nicht erreichbar" (Fehlercode 1), aber kein "did not start … compile/syntax error". Ein Kompilierfehler heißt: `qnear` wird auf 3.3 nicht angenommen – dann vor T5b `mauer/ur/script.py` anpassen.
1. **Mechanik.**
   - UR-Fußschrauben fest; das Anzugsmoment steht in den Montageunterlagen, nicht im Repo.
   - Kameraadapter 018660_A_1: Sitz der M6 prüfen (9 Nm laut UR-Handbuch, input/kamerahalter_handoff.md). Für dieses gedruckte Teil ohne Hülsen gilt: nach dem Nachziehen 24–48 h warten, dann erst kalibrieren ([camera.adapter]-Kommentar). Nur bei lockeren Schrauben nachziehen.
   - Kabel mit Zugentlastung.
   - Versorgung des Greifers über ARES-24 V prüfen. Die Kamera hängt am Werkzeugstecker: `py.exe tools/ur_check.py tool-voltage 24`.
2. **UR-Lage ([ur5]).**
   - mount_x und mount_y: Abstand der UR-Fußmitte zur ARES-Vorderkante (x = +560 mm, [ares] length 1120) und zu beiden Seitenkanten (y = ±300 mm). Dann gilt mount_x = 560 − Abstand vorn und mount_y = (Abstand rechts − Abstand links) / 2, mit y nach links positiv. Vorher prüfen, ob die gemessene Kante der CAD-Kontur entspricht.
   - mount_z: Höhe der UR-Fußauflage über der Deckoberseite, also eine eventuelle Adapterplatte. mount_z = 333,6 mm + Plattendicke. Die Deckoberseite über dem Boden am UR messen und mit [ares] deck_top_z vergleichen (gefedert).
   - mount_rz: Am Pendant im Move-Tab Feature "Base", Speed-Slider niedrig, den TCP 300 mm in +X verfahren. Zuerst prüfen, dass +X nach ARES-vorn zeigt. Zeigt es zur Seite oder nach hinten, ist mount_rz ±90° oder 180°; das ist eine Konfigurationsänderung mit vollständiger Neuplanung. Dann die seitliche Abweichung des Wegs gegen eine Kante parallel zu ARES-x messen: mount_rz = atan(Abweichung / 300 mm).
3. **Magazin ([deck]).** Halterhöhe über dem Deck messen (holder_z). Die Haltermitten zur UR-Fußachse messen: Reihen bei x = −653,6 und −433,6 mm, Spalten bei y = −205 / 0 / +205 mm, Halbsteinplatz r1y0 bei y = −255 und −155 mm (Anhang A.5). Optional mit dem Roboter: den TCP am Pendant langsam mit Sicherheitsabstand über einen Halter fahren und `py.exe tools/ur_check.py pose` lesen. Der Sollwert ist die ARES-Koordinate aus A.5 minus die UR-Montage. Abweichungen über 2 mm (**Vorschlag**) gehen in die Konfiguration. Die Magazin-Picks liegen außerhalb der Kameraregelung.
4. **Kamera-Montagecheck.** calib-Board flach im Bild, Kamera etwa 320 mm darüber: `py.exe tools/measure_target.py mount --boards calib --report data/mount_check/mount_<Datum>_ares.json`. Nach "yes" fährt der TCP 10 mm in Basis-+x und zurück, dann 10 mm in +y und zurück, mit 20 mm/s.
5. **Massen und Halbstein.**
   - Je 3 Voll- und Halbsteine wiegen und den größten Wert eintragen (**Vorschlag**, konservativ).
   - Halbstein vermessen: Länge, Breite und Höhe des Körpers, Mittenabstand des Zapfenpaars quer, Zapfenlänge.
   - Nutzlast prüfen: 1,68 kg Werkzeug ([ur] payload_tool_kg, CONFIRMED) plus Stein ≤ 5,0 kg (UR5-Nennlast, `preflight_real`), also Vollstein ≤ 3,32 kg. Der Schwerpunkt liegt etwa 207 mm vor dem Flansch (tcp_z 147 + halbe Steinhöhe 60); mit dem Nutzlastdiagramm im UR5-Handbuch vergleichen.
   - [ur] payload_cog_mm bestimmen, durch Auswiegen des Werkzeugs oder aus Teilmassen, und die Methode notieren.
6. **Greifer.**
   - `py.exe tools/ur_check.py grip open` und `grip close` (DO0 / DO1, Puls 0,5 s).
   - Greiftest: einen Stein (Zapfen oben) auf einen Halter stellen. Die Pendant-Nutzlast vorübergehend auf Werkzeug + Stein setzen. Den offenen Greifer langsam über den Stein fahren; der Greiferkörper darf die Zapfen nicht berühren.
   - `grip close`, 50 mm anheben, 10 s halten: Der Stein darf nicht rutschen. Das Schließen auf dem Stein in Zeitlupe filmen; daraus folgt [ur] grip_wait_s.
   - Absetzen, `grip open`, Pendant-Nutzlast wieder auf 1,68 kg.
7. **Parkpose.** [ur] park_q_deg = [161.83, −91.86, 41.77, −39.91, −90.0, 161.83]°.
   - Am Pendant die Gelenkwerte eingeben und die Anfahrtaste gedrückt halten (CB3: "Auto"; Loslassen stoppt). Speed-Slider niedrig, Weg beobachten.
   - In der Pose prüfen: Backen frei vom Unterarm, Greifer und Kamera frei von Deck und Magazinhaltern. Der TCP steht etwa 919 mm über dem Boden und 144 mm vor der ARES-Front.
   - Den Arm in der Parkpose lassen. (Steht der Arm genau in der Parkpose – innerhalb 0,01 rad je Gelenk –, fragt der Sequencer später kein "robot: park" an.)
8. **Optional:** ARES-Neigung in der Parkpose notieren (Wasserwaage auf dem Deck), als Referenz für T7.

**Bestanden, wenn:**
- Die IK-Prüfung aus Schritt 0 läuft ohne Kompilierfehler.
- Alle Werte sind mit Methode gemessen.
- +X der UR-Basis zeigt nach vorn, oder die tatsächliche Lage ist erfasst.
- `measure_target` meldet "MOUNT OK" (≤ 5°; am 2026-10-06: 3,4°).
- Die Nutzlast liegt bei ≤ 5,0 kg.
- Der Greifer hält den Stein ohne Rutschen.
- Die Parkpose ist ohne Kontakt erreicht.
- Die Magazinhalter liegen in der Toleranz, oder die Änderung ist für den Freeze vorgemerkt.

**Protokollieren:** Ausgabe von `plan --check`, Tabelle 6.4 mit allen Werten und Methoden, Mount-Report, Fotos von UR-Fuß, Adapter und Parkpose, Video des Greiftests.

**Sicher abbrechen:** Pendant-Taste loslassen oder Stop drücken. Ctrl-C bricht den laufenden Block von `measure_target` ab. Bei Gefahr den UR-Not-Halt.

---

### T4 Hand-Auge-Kalibrierung auf ARES

**Zweck:** `T_flange_cam` (`calib/handeye.json`) wird auf ARES neu bestimmt und unabhängig bestätigt. Dabei zeigt sich, ob sich die Kamera beim Umbau bewegt hat.

**Voraussetzungen:**
- T3 bestanden; ab jetzt keine Arbeit an Kamera, Adapter, Fokus oder Blende.
- Die Kamera ist frei.
- Die Testbeleuchtung ist an.
- Das calib-Board ist starr mit ARES verbunden, auf dem Deck oder auf einer starren Platte über den Magazinhaltern, ohne Steine im Magazin. Die Lage ist beliebig, solange die Kamera es aus etwa 320 mm sieht; `orbit` braucht [boards.calib] xyz nicht.
- Weil das Board auf ARES liegt, hebt sich die Schwingung von ARES heraus.

**Ablauf:**
1. Belichtung: `py.exe tools/cam_check.py live`, Board im Bild. Mit `+`/`-` einstellen, bis kaum gesättigte Pixel (rot, < 1 %) bleiben. Den Wert für [camera] exposure_us notieren (er ändert die Kalibrierung nicht). Mit `q` beenden.
2. Startpose per Freedrive oder Pendant: Kamera etwa 320 mm über dem Board (Schärfebereich 290–355 mm), Board etwa mittig, Kamera etwa senkrecht. **Achtung:** `orbit` prüft nur den Abstand zur Boardebene und das Werkzeug gegen den Arm, **nicht** ARES-Aufbauten oder Magazinhalter. Die Umgebung beobachten.
3. Lauf 1: `py.exe tools/calib_handeye.py orbit --dataset data/he_<Datum>_ares1`. Die Liste der behaltenen und verworfenen Ansichten lesen, dann "yes" tippen. Es folgen bis zu 25 Ansichten bei 50 mm/s (Neigung 15–25°, Rollen bis 40°, Abstand ±20 mm) und die Rückkehr zur Startpose.
4. Hat sich die Kamera bewegt? `py.exe tools/handeye_solve.py data/he_<Datum>_ares1 --check calib/handeye.json`. Maßgeblich ist die **Streuung** des Boards über die Ansichten. Das in der alten Datei gespeicherte Board lag auf dem Labortisch; die Abweichung zu ihm sagt nichts.
5. Lösen: zuerst `py.exe tools/handeye_solve.py data/he_<Datum>_ares1 --dry-run`, dann `py.exe tools/handeye_solve.py data/he_<Datum>_ares1 --out calib/handeye.json --force`. Die alte Datei bleibt in git. Die Ausgabe zeigt alle fünf OpenCV-Verfahren, ihre Uneinigkeit, den Hold-out (jede 5. Ansicht) und Warnungen.
6. Lauf 2 von einer anderen Startpose (anderer Abstand oder andere Seite, Board unberührt): `py.exe tools/calib_handeye.py orbit --dataset data/he_<Datum>_ares2`.
7. Unabhängige Prüfung: `py.exe tools/handeye_solve.py data/he_<Datum>_ares2 --check calib/handeye.json`. Jetzt liegt das gespeicherte Board an derselben Stelle; Abweichung und Streuung zählen beide.
8. Arm in die Parkpose bringen und `calib/handeye.json` committen.

**Bestanden, wenn** (Schwellen sind **Vorschlag**, abgeleitet aus den Tischwerten vom 2026-10-06):
- Lauf 1 liefert ≥ 20 verwertbare Ansichten (am Tisch: 25 Ansichten, 20 zum Lösen und 5 Hold-out).
- TSAI, PARK und HORAUD liegen ≤ 1 mm / 0,1° auseinander (CAMERA_SETUP §7). ANDREFF und DANIILIDIS wichen schon am Tisch weiter ab (3,4 mm / 0,23° bzw. 0,14 mm / 0,10°, mit Warnung von `solve`); ihre Abweichung wird notiert, ist aber kein Abbruchgrund.
- Hold-out ≤ 1,0 mm / 0,5° RMS (am Tisch: 0,85 mm / 0,48°).
- Prüfung von Lauf 2: Das Board liegt ≤ 1,0 mm / 0,5° von der Kalibrierung, Streuung RMS ≤ 1,0 mm (am Tisch: 0,65 mm / 0,33°, RMS 0,89, max. 1,40 mm).
- Der Unterschied zur Tischkalibrierung ist notiert. Tischwert: xyz (−149,5; 3,0; 46,2) mm, rpy (−1,14; 0,46; −0,42)°. Mehr als 3 mm / 0,5° heißt, dass sich die Kamera bewegt hat; das ist zulässig, wenn Lauf 1 und Lauf 2 übereinstimmen.

**Protokollieren:** Beide Datensätze sichern, Ausgaben von `solve` und `--check`, Belichtung, Commit der Kalibrierung.

**Sicher abbrechen:** Ctrl-C bricht den laufenden Block ab, sonst Stop am Pendant oder UR-Not-Halt. Ein abgebrochener Lauf wird mit einem neuen `--dataset`-Ordner wiederholt.

---

### T5a Bodenaufbau und Boards von Stop 0 (Phase 1)

**Zweck:** Leitstreifen, Board-Platten und Station liegen so, wie die Konfiguration sagt. W0 und W1 sind von Stop 0 aus messbar und passen zueinander. Die Einschwingzeit von ARES ist gemessen.

**Voraussetzungen:** T4 bestanden; Bodenaufbau nach 3.3 fertig; der Arm steht in der Parkpose.

**Ablauf:**
1. **Board-Drucke:** Auf W0–W6, S0 und S1 mit dem Messschieber über 5 Felder messen (Soll 80,0 mm bei 16-mm-Feldern). Daraus [boards.ref] square_mm, und marker_mm mit demselben Faktor. Weichen die Boards untereinander um mehr als 0,1 % (0,08 mm auf 80 mm) ab, notieren: Die Konfiguration kennt nur einen Wert für alle.
2. **Bodenmaße** nach targets/guides/floor_plan.md:
   - Markierungspunkte und Diagonalen;
   - Streifenbreite 120 mm ([plates] block_width), Teilung der V-Laschen 200 mm ([plates] block_length), MDF-Dicke 4 mm ([wall] base_z);
   - an den Eck-L-Stücken sitzt B 2,69 mm von A ab (Rippe 1,69 + Fuge 1,0 mm);
   - Stationsecken;
   - jede Platte liegt mit beiden V-Kerben auf ihren Laschen, die Kante an der Leiste.
3. **ARES auf die Startmarke von Stop 0** (Anhang A.3):
   - Den Fahrzeugumriss mit Klebeband markieren und ARES mit Jog oder Relativfahrt durch die offene Seite in die C fahren. Die Räder halten mindestens 50 mm Abstand zu Leisten und Platten ([routes] clearance_mm).
   - Ziel: rechte Fahrzeugseite 220 mm von der ARES-seitigen Leistenkante von A, vorn und hinten gleich (±2 mm über 1120 mm ≈ ±0,1°). Die Vorderkante steht bei x = 720 mm im Kartenrahmen.
4. **W0 messen:** `py.exe tools/cam_check.py live` starten, die Kamera per Freedrive etwa 320 mm über W0 bringen (Board in der Bildmitte), mit `q` beenden. Dann `py.exe tools/measure_target.py repeat --boards W0 --n 10 --report data/T5/W0_repeat.json`. Mittelwerte für xyz und rpy im UR-Basis-KS sowie die Streuung notieren.
5. **W1** genauso: `py.exe tools/measure_target.py repeat --boards W1 --n 10 --report data/T5/W1_repeat.json`.
6. **Auswerten:**
   - Abstand der Board-Ursprünge W0–W1, Soll 600,0 mm (W0 und W1 bei x = 60 / 660 mm im Schenkelrahmen A).
   - Gierwinkel-Differenz etwa 0.
   - z beider Boards gleich.
   - Gegenprobe der UR-Höhe: z_Board ≈ 4,1 mm − mount_z. Das schließt die Einfederung ein.
7. **Einschwingzeit:** ARES schwingt auf den Federn gegen den Boden; das Deck-Board aus T4 sieht das nicht. Über W0: `py.exe tools/measure_target.py repeat --boards W0 --n 10 --settle-s 0,0.5,1,2 --move-mm 20 --report data/T5/W0_settle.json`, dann "yes" tippen. [camera] settle_s ist die kleinste Zeit, ab der die Streuung nicht mehr sinkt. Optional mit `--move-mm 100` für stärkere Anregung.
8. Den Arm per Pendant in die Parkpose fahren. ARES bleibt auf der Marke.

**Bestanden, wenn:**
- Die Diagonalen liegen innerhalb ±2 mm (**Vorschlag**), und alle Platten sitzen.
- W0 und W1 sind mit ≥ 8 Ecken und RMS ≤ 1 px gemessen ([vision] min_corners, max_reproj_px).
- Der Abstand W0–W1 beträgt 600 ± 1 mm (**Vorschlag**: damit bleibt der Fit-RMS deutlich unter [sequencer] max_fit_rms_mm 1,0 mm).
- Die Gierwinkel-Differenz ist ≤ 0,2° (**Vorschlag**).
- settle_s ist bestimmt.

**Protokollieren:** Maße, Diagonalen und Reports; Fotos des Aufbaus von festen Standpunkten; Lage von ARES an der Marke.

**Sicher abbrechen:** Ctrl-C bei `measure_target`, Stop am Pendant, HALT für ARES-Fahrten.

---

### T5b Wandrahmen durch den Sequencer, erste ARES-Fahrt aus der HMI (Phase 2)

**Zweck:** Der Sequencer misst W0 und W1 an Stop 0 und passt den Wandrahmen ein. Dann korrigiert er die ARES-Lage mit einer kleinen Fahrt über seine eigene ADS-Verbindung im HMI-Prozess. Das ist der erste reale Test von Bedienmuster A in einem Prozess.

**Voraussetzungen:**
- Freeze erledigt ("REAL preflight ok"), T5a bestanden.
- Magazin nach der Startfüllung aus Anhang A.5 bzw. der neuen Steinliste.
- Station voll (16 Steine, A.6).
- Backen leer und offen, Arm in der Parkpose, Speed-Slider niedrig.

**Ablauf:**
1. `py.exe -m hmi --ares --job data/jobs/nominal_C_robodk.json` und MANUAL herstellen (ARES-control-Tab).
2. ARES auf die Startmarke von Stop 0 stellen (Jog oder Relativfahrt). Dann im ARES-control-Tab Relativfahrt **↓ Back** 30 mm bei 50 mm/s, **GO** zweimal. ARES steht so absichtlich 30 mm hinter der Sollage; das bleibt unter [sequencer] max_jump_mm von 50 mm. Warten, bis ARES steht (Start ist gesperrt, solange ARES fährt).
3. Mauer-Tab: REAL, **`stops 0 to 3`**, Schrittmodus an, camera loop an, save images an. **Prepare / Connect**; die Anzeige lautet "REAL preflight ok".
4. **Checkliste vor Start** (die HMI fragt das noch nicht ab, Anhang C1):
   - ☐ Backen leer
   - ☐ Magazin wie Liste, alle anderen Plätze leer
   - ☐ Station voll
   - ☐ niemand im Arbeitsraum
   - ☐ beide Not-Halte besetzt
   - ☐ Speed-Slider niedrig
   - ☐ HMI ist das aktive Fenster (kein rotes Banner)
5. **Start.** Die HMI prüft den Preflight unmittelbar vor der ersten Bewegung noch einmal (die Liste wird dabei neu geschrieben). Die Bestätigungen erscheinen in der Leiste über den Tabs. Steht der Arm genau in der Parkpose, gibt es kein anfängliches "robot: park":
   - `robot: look stop0-W0 (W0)`: Go. Das ist die erste Bewegung, die der Motion Guard plant. Camera-Tab: W0 ist grün (accepted), mit Eckenzahl und RMS in px.
   - `robot: look stop0-W1 (W1)`: Go.
   - Camera-Tab "Last frame fit": Fit-RMS, Fehler gegen Soll etwa 30 mm, Sprung < 50 mm. Wall-pose-Tab: gemessene ARES-Lage.
   - `robot: park`: Go (ARES fährt nur bei geparktem Arm).
   - `ARES translate dx ≈ +30 mm, dy … (stop 0 correction 1)`: Go. Bei einem Kursfehler über 0,5° kommt davor `ARES rotate … deg (stop 0 correction 1)`. ARES fährt; im ARES-control-Tab läuft die Fahrt und endet mit Ergebnis 1, im Wall-pose-Tab erscheint die Live-Lage.
   - Erneute Blickposen `robot: look stop0-W0 (W0)` und `robot: look stop0-W1 (W1)`: Go. Der Fehler ist danach ≤ 20 mm, es folgt keine weitere Korrektur.
   - `robot: park`: Go.
   - `robot: pick magazine slot r1y2l3 (full)`: **Decline**. Der Lauf steht auf "aborted". Die HMI bleibt offen.
6. Schlägt der Sequencer keine Korrektur vor (Fehler < 20 mm): Decline, **Release**, den Versatz auf 40 mm erhöhen und wiederholen.

**Bestanden, wenn:**
- Beide Boards sind accepted (≥ 8 Ecken, RMS ≤ 1 px).
- Fit-RMS ≤ 1,0 mm.
- Der erste Sprung entspricht etwa dem eingestellten Versatz und bleibt unter 50 mm / 3°.
- Die Korrekturfahrt endet mit Ergebnis 1, ohne Heartbeat-Abbruch (26), PLC HB bleibt stabil.
- Danach ≤ 20 mm und ≤ 0,5° ([sequencer] stop_tol_mm, rotate_threshold_deg).
- Keine Warnung "continuing with the measured frame".
- Park- und Blickbewegungen ohne Kontakt.

**Protokollieren:** Run-Log-Ordner `data/runs/<Zeit>_hmi_real/`, Screenshots von Camera-Tab (Fit) und Wall-pose-Tab, Video der ersten Bewegungen, Tabelle 6.6.

**Sicher abbrechen:** Decline wirkt sofort. HALT stoppt die ARES-Fahrt mit Rampe und stoppt den UR; eine danach noch geplante Bewegung wird nicht mehr gesendet. Nach HALT während der Korrekturfahrt: Lage prüfen, dann **Confirm pose** oder **Set pose** (Abschnitt 5).

---

### T6 Ein Stein Magazin → Wand (Schrittmodus)

**Zweck:** Der erste vollständige Pick-and-Place eines Vollsteins mit dem realen System.

**Voraussetzungen:** T5b bestanden, derselbe Lauf ("aborted" am ersten Pick). Die Kegel für Stein Ac0i0 sitzen, die Backen sind leer.

**Ablauf:**
1. **Resume.** Nach jedem Resume misst der Sequencer die Wand neu: `robot: look stop0-W0 (W0)`, `robot: look stop0-W1 (W1)`, `robot: park`, jeweils Go, bei Bedarf eine Korrektur mit erneuten Blickposen und `robot: park`. (Ein `robot: park` vorweg kommt nur, wenn der Arm nicht in der Parkpose steht.)
2. `robot: pick magazine slot r1y2l3 (full)`: Go. Beobachten:
   - Anfahrt 150 mm über dem Platz (approach_mm des Jobs);
   - die letzten 60 mm mit 20 mm/s ([ur] contact_mm, v_contact);
   - Backen schließen (DO1-Puls 0,5 s + 0,5 s Wartezeit);
   - Anheben, ohne Nachbarsteine mitzunehmen.
   - UR-Tab: Zeile "jaws" zeigt den Stein, Nutzlast "tool 1.68 kg …; with the full stone … kg".
3. `robot: place stone Ac0i0 (u 100 mm, top 124 mm)`: Go. Beobachten: Transfer, Abstieg über die beiden Kegel, Öffnen, Rückzug, ohne den Stein mitzunehmen.
4. Die nächste Bestätigung `robot: pick magazine slot r1y0al1 (half)`: **Decline**.
5. Bei gestopptem Lauf prüfen:
   - Der Stein sitzt auf beiden Kegeln und liegt ohne Spalt auf dem MDF (Fühlerlehre).
   - Die Rippen stehen beidseitig etwa gleich weit über die Leiste ([brick] rib_mm 1,69).
   - Die Stirnseite liegt am Schenkelanfang.
   - Fotos machen.

**Bestanden, wenn:**
- Es gab keinen Protective Stop und keine Ablehnung durch den Motion Guard.
- Der Stein wurde abgesetzt, ohne zu drücken und ohne zu fallen.
- Er sitzt ohne Spalt auf beiden Kegeln.
- Der Greifer hat ihn freigegeben.
- Das Magazin ist sonst unverändert.

**Protokollieren:** Run-Log, Video von Pick und Place, Prüfblatt 6.5, Zeit pro Stein (Kachel "time per stone").

**Sicher abbrechen:**
- Vor jedem Go: Decline. Während einer Bewegung: HALT.
- Stein im Greifer nach HALT oder Decline:
  1. Stein mit der Hand halten.
  2. Greifer am Pendant öffnen (I/O-Tab, DO0-Puls).
  3. Stein ablegen.
  4. Arm per Pendant frei fahren.
  5. **Jaws empty** (setzt auch die Nutzlast des UR auf das Werkzeug zurück), dann **Resume** (läuft im Schrittmodus).

---

### T7 Ein vollständiger Stop – Stop 0, Teil 1 (14 Steine, bis das Magazin leer ist)

**Zweck:** Der Roboter setzt alle Steine einer Magazinfüllung, mit allen Lagen und den ersten Halbsteinen.

**Voraussetzungen:** T6 bestanden, derselbe Lauf.

**Ablauf:**
1. **Resume**, Neumessung, dann die Steine in der Reihenfolge aus Anhang A.7: Ac1i0h (Halbstein, Lage 1), Ac0i1, Ac1i1, Ac2i0, Ac3i0h, Ac0i2, Ac1i2, Ac2i1, Ac3i1, Ac0i3, Ac1i3, Ac2i2, Ac3i2.
2. Der Schrittmodus bleibt an, bis je einmal erfolgreich gesetzt sind: ein Halbstein auf Lage 1 (Ac1i0h), ein Stein auf Lage 2 (Ac2i0), Lage 3 mit Halbstein (Ac3i0h). Danach darf man ihn ausschalten; die Checkbox wirkt sofort. Die Hand bleibt an HALT.
3. Je Stein prüfen: Die Zapfen greifen. Die Lagerfuge ist etwa 1 mm und gleichmäßig ([brick] bed_joint: die Zapfen sind 1 mm länger als die Buchsen tief). Kein Verkanten. Beim Halbstein sitzt ein Zapfenpaar mittig.
4. Die Neigung von ARES beobachten, vor allem bei weiten Ablagen (u 500–700 mm) mit fast leerem Magazin.
5. Nach Ac3i2 ist das Magazin leer. `robot: park`: Go. Dann `ARES translate dx ≈ −1260 mm, dy ≈ +200 mm (stop 0 -> station leg 0 (dead reckoning))`: **Decline**.
6. Bei gestopptem Lauf: Lagerfugen mit der Fühlerlehre, Flucht der ARES-seitigen Wandfläche mit dem Richtscheit, Treppenform wie geplant.

**Bestanden, wenn:** 14 von 14 Steinen sind gesetzt und sitzen. Es gab keine Ablehnung durch den Motion Guard und keinen Protective Stop. Alle Warnungen im Run-Log sind erklärt. ARES hat weder abgehoben noch gekippt. Die Zeit pro Stein ist notiert.

**Protokollieren:** Run-Log, Prüfblatt 6.5 je Stein, Fotos, Neigung.

**Sicher abbrechen:** Pause wirkt erst an der nächsten Grenze mit leeren Backen; eine laufende Ablage wird fertig. Decline wirkt sofort, ebenso HALT und Not-Halt. Abbruchkriterien siehe 2.4.

---

### T8 Eine Stationsfahrt (Reload 1) und der Rest von Stop 0

**Zweck:** ARES fährt die geprüfte Route zur Station und misst S0 und S1. Der Roboter lädt 16 Steine ins Magazin um. ARES fährt zurück, misst die Wand neu, und Stop 0 wird fertig.

**Voraussetzungen:**
- T7 bestanden, derselbe Lauf.
- Station voll (A.6).
- Fahrbereich frei: der Weg aus der C über die offene Seite zur Station, und um die Drehpunkte (Kartenrahmen: Hinweg x −1100, y 840; Rückweg von Stop 0 x −500, y 740) je mindestens 0,8 m Radius (Schätzung wie in T2b).
- Das Scannerverhalten aus T0 ist bekannt.

**Ablauf (erwartete Bestätigungen):**
1. **Resume.** Die unterbrochene Route läuft weiter, ohne Neumessung an Stop 0:
   - `ARES translate … (stop 0 -> station leg 0 (dead reckoning))`: Go, etwa 1,28 m schräg rückwärts (dx −1260, dy +200 mm);
   - `ARES rotate +90.00 deg (stop 0 -> station leg 1 (dead reckoning))`: Go;
   - `ARES translate … (stop 0 -> station last leg (closed loop))`: Go, etwa 60 mm bis zum Dock.
2. `robot: look station-S0 (S0)`, `robot: look station-S1 (S1)`, `robot: park`: Go. Ist der Dockfehler größer als 30 mm ([sequencer] dock_tol_mm), folgen `ARES translate … (dock correction 1)`, erneute Blickposen und `robot: park`.
3. 16-mal `robot: pick station slot …` und `robot: place magazine slot …`: Go. Die obere Lage kommt zuerst. Die Steine sitzen in den Magazinstapeln.
4. `robot: park`, dann die Rückroute mit Go je Schritt:
   - `ARES translate … (station -> stop 0 leg 0 (dead reckoning))` rückwärts und seitlich,
   - `ARES rotate -90.00 deg (station -> stop 0 leg 1 (dead reckoning))`,
   - `ARES translate … (station -> stop 0 leg 2 (dead reckoning))`,
   - `ARES translate dx ≈ +20 mm … (station -> stop 0 last leg (closed loop to 40 mm before the stop))`: die letzte Teilfahrt endet 40 mm vor dem Stop ([sequencer] arrival_standoff_mm).
5. Blickposen W0 und W1, `robot: park`, danach Korrekturen. Nach der Station gelten 100 mm / 5° als Sprunggrenze. Eine Drehkorrektur um etwa 1° ist zu erwarten: E003 zeigte etwa 1,2° Verlust nach Umkehr der Drehrichtung (`mauer/ares/ads.py`).
6. Die restlichen 10 Steine: Ac0i4, Ac1i4, Ac2i3, Ac3i3, Ac0i5h, Ac1i5, Ac2i4, Ac3i4, Ac2i5h, Ac3i5. Neu ist der Halbstein auf Lage 0 am Eckende (Ac0i5h); hier mit Schrittmodus arbeiten.
7. Stop 0 ist fertig. `robot: park`, dann `ARES translate … (stop 0 -> stop 1 leg 0 (dead reckoning))`: **Decline**. Damit endet T8.
8. Optional, nur wenn alle Fahrten bisher unauffällig waren, eine Wiederanlauf-Übung:
   1. In Schritt 4 während der ersten Rückfahrt HALT drücken. Erwartet: Ergebnis 20, Lauf "error"; die Zeile "ARES" im Mauer-Tab endet auf "odometry", Resume meldet "the ARES pose is unverified".
   2. Die ARES-Lage am Boden prüfen, dann **Confirm pose**.
   3. **Resume** im Schrittmodus. Die Route läuft ab der unterbrochenen Teilfahrt weiter.

**Bestanden, wenn:**
- Jede Teilfahrt endet mit Ergebnis 1.
- Es gab keinen Kontakt mit Leisten, Platten, Steinen oder Station, und kein Rad lief über MDF.
- S0 und S1 sind accepted, der Dockfehler ist nach höchstens 2 Korrekturen ([sequencer] max_corrections) ≤ 30 mm.
- Alle 16 Steine sind umgeladen.
- Der Sprung bei der Rückkehr bleibt ≤ 100 mm / 5°, danach ≤ 20 mm / 0,5°.
- Stop 0 ist komplett: 24 von 24 Steinen sitzen.

**Protokollieren:** Dockfehler, Sprünge und Korrekturen (Wall-pose-Tab "Camera fits", Tabelle 6.6), kleinster beobachteter Abstand zur Station am Dock, Scanner-Ereignisse, Dauer der Stationsfahrt, Run-Log.

**Sicher abbrechen:** HALT stoppt ARES mit Rampe; danach **Confirm pose** oder **Set pose** (Abschnitt 5). Löst der Frontscanner am Dock aus (Ergebnis 23):
1. Im ARES-control-Tab **Re-arm Safety** bzw. **Safety Run** → **AMR Reset** → **Start** → **Manual**.
2. Die Lage prüfen, **Confirm pose**.
3. **Re-check**, dann **Resume**.

---

### T9 Beinwechsel A → B

**Zweck:** ARES wechselt von Schenkel A zu Schenkel B. Die Route hat 2 Teilfahrten, die letzte endet 40 mm vor Stop 1. Dort misst der Sequencer W2 und W4 mit den Sprunggrenzen nach einer Route, korrigiert auf Stop 1 und setzt die ersten Steine von B an der Ecke zu A.

**Voraussetzungen:** T8 bestanden, derselbe Lauf (Decline an der ersten Fahrt des Beinwechsels). Die Station ist wieder voll befüllt, 16 Steine; nach 6 Steinen von B folgt Reload 2.

**Ablauf:**
1. **Resume**, dann (der Arm steht seit T8 in der Parkpose, also ohne `robot: park`):
   - `ARES translate dx ≈ −20 mm, dy ≈ +22.7 mm (stop 0 -> stop 1 leg 0 (dead reckoning))`: Go;
   - `ARES translate dx ≈ +20 mm … (stop 0 -> stop 1 last leg (closed loop to 40 mm before the stop))`: Go.
2. `robot: look stop1-W2 (W2)`, `robot: look stop1-W4 (W4)`, `robot: park`: Go. Die Sprunggrenzen nach einer Route sind 100 mm / 5°.
3. Korrektur auf Stop 1, etwa 40 mm vorwärts (`… (stop 1 correction 1)`): Go. Neumessung, `robot: park`, danach ≤ 20 mm / 0,5°.
4. Im Schrittmodus die Steine Bc0i0 (Lage 0 an der Ecke), Bc1i0h, Bc0i1, Bc1i1, Bc2i0, Bc3i0h. Nach Bc0i0 die Fuge zwischen der Stirnseite von B und den Rippen der Innenseite von A messen (Soll [wall] corner_gap_mm 1,0 mm, ASSUMPTION) und prüfen, dass der Stein auf seinen Kegeln sitzt.
5. Das Magazin ist leer, die Leiste zeigt "Pick-up station empty or short of the next stone types …". Die Station wurde vorher gefüllt, trotzdem **Refilled** drücken: Erst damit zählt der Sequencer die Station als voll. Dann `robot: park` und `ARES translate … (stop 1 -> station leg 0 (dead reckoning))`: **Decline**. Damit endet T9.

**Bestanden, wenn:**
- Die Teilfahrten enden mit Ergebnis 1, ohne Kontakt mit dem gebauten Schenkel A oder den Platten von B.
- W2 und W4 sind accepted, der Sprung bleibt ≤ 100 mm / 5°, nach der Korrektur ≤ 20 mm / 0,5°.
- Der erste Stein von B ist ohne Kollision mit A gesetzt, die Eckfuge ist gemessen (Abweichung von 1,0 mm notiert).
- 6 von 6 Steinen sitzen.

**Protokollieren:** wie T8, dazu die Eckfuge (oben und unten, Fühlerlehre) und Fotos der Ecke.

**Sicher abbrechen:** wie T8. **Keinen neuen Lauf "ab Stop 1" starten, solange ARES an Stop 0 steht.** Ein neuer Lauf nimmt ARES an der Startmarke des Start-Stops an und fährt den Beinwechsel nicht (Abschnitt 5).

---

### T10 Ganze Mauer

**Zweck:** Die C wird vollständig gebaut: 76 Steine, 4 Stops, 5 Nachladungen, 4 Stationsauffüllungen.

**T10a – Fortsetzung des Laufs aus T6–T9 bis zum Wandende:**
1. **Resume**. Es folgen:
   - Reload 2 (12 Vollsteine);
   - der Rest von Stop 1: 12 Steine, Reload 3, 4 Steine;
   - Stop 1 → 2 (Route 327 mm, W2/W4), dann Stop 2 mit 8 Steinen;
   - **zweiter Beinwechsel** Stop 2 → 3 (B → C, 189 mm, W5/W6);
   - Stop 3 mit 22 Steinen, Reload 4 und 5.
2. Den Schrittmodus an jeder neuen Situation wieder einschalten: erste Fahrt Stop 1 → 2, Beinwechsel B → C, erster Stein von C an der Ecke zu B, Halbsteine an den freien Enden.
3. Bei jeder Stationsabfrage die Station nach Liste füllen und **Refilled** drücken.

**T10b – Gesamtlauf aus leerem Zustand (Abnahme):**

*Voraussetzungen:*
- T10a bestanden.
- Alle Steine abgebaut; Kegel, Leisten und Platten unverändert (Sichtprüfung, kein Board verschoben).
- Parameteränderungen aus T5b–T10a nur mit neuem Stempel (3.5).
- Steine laut Steinliste bereit.
- ARES auf der Startmarke, ohne Versatz.
- Magazin mit der Startfüllung, Station voll.

*Ablauf:*
1. Neue HMI-Sitzung: REAL, `stops 0 to 3`, Schrittmodus **aus**, camera loop an, save images an. **Prepare / Connect** bis "REAL preflight ok". Checkliste aus T5b Schritt 4, dann **Start**.
2. Der Bediener bleibt an HALT, die Sicherheitsperson an den Not-Halten. Stationsabfragen mit **Refilled** beantworten. Von einem festen Standpunkt filmen.
3. Am Ende steht der Zustand auf "done". **Open summary** öffnet `hmi_summary.json` (während des Laufs gesperrt). Dann **Release**.

**Bestanden, wenn (T10b):**
- 76 von 76 Steinen sind gesetzt und sitzen, der Zustand ist "done", mit 5 Nachladungen und 4 Auffüllungen.
- Es waren kein HALT und kein Not-Halt nötig. Es gab keine Ablehnung durch den Motion Guard und keinen Protective Stop.
- Alle Fits liegen bei ≤ 1,0 mm RMS. Es gab keine Warnung "continuing with the measured frame" oder "correction … capped".
- ARES hat den Aufbau nicht berührt.
- Mauergeometrie:
  - Oberkante des Körpers von Lage 3 bei 487 mm (laut Job) an allen Ecken;
  - die Schenkel fluchten;
  - Ecken wie geplant;
  - das freie Ende von C steht 22,7 mm über den Anfang von A hinaus (Konfiguration).

**Protokollieren:** `hmi_summary.json`, `run.jsonl`, Gesamtzeit, Zeit pro Stein, Zahl der Korrekturen, größter Fit-RMS, Endfotos von festen Standpunkten, Mauermaße (Höhen an den Ecken, Fluchten), Prüfblatt 6.5.

**Sicher abbrechen:** Für geplante Unterbrechungen **Pause** (wirkt an der nächsten Grenze mit leeren Backen). Bei Gefahr HALT oder Not-Halt. Die HMI nicht schließen, denn der Wiederanlauf-Zustand liegt nur im Speicher.

---

## 5 Störungen und Wiederanlauf

| Anzeige / Situation | Bedeutung | Maßnahme |
|---|---|---|
| Resume verweigert: "a stone may be in the jaws" | Decline, HALT oder Roboterfehler zwischen Pick und Place | Stein halten, Greifer öffnen (Pendant, I/O-Tab, DO0-Puls), Stein ablegen, Arm frei fahren → **Jaws empty** → **Resume** |
| "the ARES pose is unverified" | Eine ARES-Fahrt endete nicht ok; die Lage stammt aus der Odometrie | ARES am Boden prüfen → **Confirm pose**, sonst **Set pose** (x, y, θ im Wandrahmen, Anhang A.2) |
| "the ARES pose is unknown" | ARES-Fehler ohne Ergebnis | **Set pose** (Confirm pose ist dann gesperrt) |
| "ARES moved by … since the run stopped" | ARES wurde in der Pause bewegt (Jog oder GO sind dann erlaubt) | **Apply odometry** oder **Set pose** |
| "cannot verify that ARES did not move" | Der ADS-Worker der HMI ist getrennt | ARES-control-Tab neu verbinden, MANUAL herstellen, **Re-check** |
| Start / Resume gesperrt: "ARES is moving (relative move or jog)" | ARES fährt noch (Relativfahrt, Jog, Auslaufen) | warten, bis ARES steht |
| "start refused: preflight: … blocking problems" | Der Preflight unmittelbar vor der ersten Bewegung hat einen Blocker gefunden (z. B. MANUAL verlassen, UR nicht RUNNING) | Ursache beheben → **Re-check** → **Start** |
| "resume refused: HALT pressed during the resume checks" (bzw. "Abort …", "start …") | HALT oder Abort während der Prüfungen von Resume / Start | nichts hat sich bewegt; Ursache klären, dann erneut **Resume** / **Start** |
| Preflight-Blocker beim Resume | z. B. MANUAL verlassen, UR nicht RUNNING | Ursache beheben → **Re-check** → **Resume** |
| "config/station.toml … changed on disk since the job was loaded" | Die Konfiguration wurde nach dem Laden des Jobs geändert | **Release**, Job neu laden, **Prepare / Connect** |
| MeasurementError, Lauf "paused" | Board nicht gesehen oder Fit-RMS > 1 mm; der Arm ist geparkt | Board verdeckt oder verschoben? Licht? Beheben → **Resume** (misst neu) |
| FrameJumpError, Lauf "error" | Die Messung weicht mehr als 50 mm / 3° (nach einer Route oder der Station 100 mm / 5°) von der Erwartung ab: falsches Board, Rutschen, Kalibrierung | Stoppen und Ursache klären; ARES-Lage prüfen, eventuell **Set pose** |
| "refused by the motion guard" | Kein freier Weg im Kapselmodell; es hat sich nichts bewegt | Nicht erzwingen; Abweichung zwischen Modell und Realität klären und dokumentieren |
| "HALT: … refused before anything was sent", Lauf "aborted" | HALT während der Planung einer Bewegung: sie wurde nicht gesendet | normal; **Resume** (Schrittmodus) |
| "RTDE lost during the block … program aborted", Lauf "error" | 5 s keine RTDE-Daten während einer Roboterbewegung; das Programm wurde gestoppt | Netz / UR prüfen; Stein im Greifer wie oben behandeln → **Re-check** → **Resume** |
| UR Protective Stop | Kraft oder Kollision | Am Pendant entsperren, Ursache prüfen, Stein im Greifer wie oben behandeln → **Resume** (Schrittmodus) |
| Not-Halt UR gelöst | – | Am Pendant Power on und Bremsen lösen → **Re-check** → **Resume** |
| ARES Safety Stop (Not-Halt, Scanner; Ergebnis 23) | – | **Re-arm Safety** / **Safety Run** → **AMR Reset** → **Start** → **Manual**; Lage prüfen, **Confirm pose** oder **Set pose**; **Re-check** → **Resume** |
| PLC lehnt eine Fahrt ab (10–13) | Parameter, nicht MANUAL, externe Steuerung, beschäftigt (Jog-Bit) | Text lesen (`MOVE_RESULT_TEXTS` in `mauer/ares/ads.py`), Ursache beheben, **Resume** |
| Ergebnis 26 "heartbeat lost" | HMI hing oder ADS war getrennt | HMI prüfen, neu verbinden, MANUAL herstellen, Lage prüfen |
| Leiste "Pick-up station empty …" (**Refilled** / **Abort run**) | Die Station hat zu wenig Steine für die nächsten Steinarten | Alle 16 Halter nach Liste füllen → **Refilled** |
| Rotes Banner "Keyboard HALT (Space / Esc) inactive" | Ein anderes Fenster ist aktiv; Leertaste / Esc erreichen die HMI nicht | in die HMI klicken; der HALT-Knopf wirkt trotzdem |
| Live stoppt, "grab failed" | Kabel, IP, Zeitüberschreitung | Netz und Kabel prüfen; **Release** → **Prepare / Connect** |
| HMI abgestürzt oder geschlossen | Der Laufzustand ist verloren. ARES stoppt (Abbruch 500 ms nach Ende des Heartbeats, MANUAL fällt nach 2 s); ein laufender UR-Block fährt zu Ende | UR bei Bedarf am Pendant stoppen. HMI mit `--ares` neu starten, MANUAL herstellen. ARES auf die Startmarke des Stops k fahren. **Teilweise gebauter Stop k:** `py.exe tools/run_job.py data/jobs/nominal_C_robodk.json --real --step --stops k: --resume-log data/runs/<Ordner>`. **Stops < k fertig, Stop k unberührt:** `… --stops k: --stop-untouched`. Das ist Muster A mit zwei Prozessen wie in E003; `run_job` nennt die Magazinbelegung und fragt Backen, Magazin und Station ab |
| Neuer Lauf ab Stop k > 0 | Der Sequencer nimmt ARES auf der Startmarke von k an und Stops < k als gebaut | ARES vorher auf die Startmarke von k fahren; die Magazinbelegung für den Neustart liefert `run_job.py` (Zeile oben) |

---

## 6 Protokollvorlagen

### 6.1 Ergebnistabelle

Ergebnis: B = bestanden, NB = nicht bestanden, A = abgebrochen. Vorschlag für die Ablage: `results/realtest_<Datum>.md`.

| Test | Datum, Zeit | Personen | Commit / Stempel-Commit des Jobs | handeye.json (created) | Ergebnis | Kennwerte | Run-Log / Datensatz | offene PLACEHOLDER | Abweichungen, Maßnahmen |
|---|---|---|---|---|---|---|---|---|---|
| T0 | | | | – | | Versionen; Not-Halt-Kopplung | – | | |
| T1a | | | | – | | SIM "done", HALT/Jaws empty ok | | | |
| T1b | | | | | | Preflight-Punkte, Banner | | | |
| T2a | | | | – | | Ergebnisse 1 / 20 / 26; +y = links | – | | |
| T2b | | | | – | | Abweichung je Fahrt (6.3) | – | | |
| T3 | | | | – | | ik_check, mount_x/y/z/rz, Massen, MOUNT OK | mount-Report | | |
| T4 | | | | | | Hold-out, Prüfung Lauf 2, Unterschied zum Tischwert | he_…_ares1/2 | | |
| T5a | | | | | | Abstand W0–W1, Gierwinkel, settle_s, Diagonalen | data/T5/ | | |
| Freeze | | | | | | Stempel ok, "REAL preflight ok" | – | | |
| T5b | | | | | | Fit-RMS, Sprung, Korrektur | | | |
| T6 | | | | | | sitzt, Zeit | | | |
| T7 | | | | | | 14/14, Zeit pro Stein | | | |
| T8 | | | | | | Dockfehler, Sprung, 24/24 | | | |
| T9 | | | | | | Sprung, Eckfuge, 6/6 | | | |
| T10a | | | | | | Stops 1–3, 2. Beinwechsel | | | |
| T10b | | | | | | 76/76, Gesamtzeit, max. Fit-RMS | | | |

### 6.2 Not-Halt-Verhalten (T0)

| Auslöser | ARES-Zustand | UR-Zustand | Quittierung, Dauer |
|---|---|---|---|
| Not-Halt ARES | | | |
| Not-Halt UR | | | |

### 6.3 ARES-Fahrten (T2b)

| Fahrt | Soll [mm / °] | Odometrie | Maßband | Abweichung | PLC-Ergebnis |
|---|---|---|---|---|---|
| vor 300 mm | | | | | |
| zurück 300 mm | | | | | |
| links 300 mm | | | | | |
| rechts 300 mm | | | | | |
| +90° / −90° (optional) | | | | | |

### 6.4 Parameteränderungen (Freeze)

| Schlüssel | alt (Status) | neu | Methode / Quelle | Test | Datum | neuer Status |
|---|---|---|---|---|---|---|
| | | | | | | |

### 6.5 Stein-Prüfblatt (T6–T10)

| Stein | Stop | Typ | gesetzt | sitzt (Kegel / Zapfen) | Lagerfuge ≈ 1 mm, gleichmäßig | Bemerkung |
|---|---|---|---|---|---|---|
| Ac0i0 | 0 | voll | | | – (Lage 0: auf MDF) | |

### 6.6 Messungen und Fits

| Ort (Stop / Dock, Anlass) | Boards | Ecken | RMS px | Fit-RMS / max. [mm] | Fehler gegen Soll | Sprung | Korrekturen |
|---|---|---|---|---|---|---|---|
| | | | | | | | |

---

## Anhang A – Daten des aktuellen Jobs

Quelle: `data/jobs/nominal_C_robodk.json` (RoboDK 6.0.0.26652, 2026-10-08 02:53: 76/76 Steine, 62 Umladungen, 13 Routen; gestempelt bei 7a98fab, seit `main` 8894751 veraltet) und sein Dry-run. Die Bestätigungstexte stammen aus einem SIM-Lauf dieses Jobs mit `main` 7ab0be8. **Nach dem Freeze neu ablesen.**

### A.1 Eckdaten

76 Steine (64 voll, 12 halb), 4 Stops, 5 Nachladungen, 4 Stationsauffüllungen. Magazin: 18 Plätze, Startfüllung 14 (12 voll, 2 halb); die oberste Lage von Reihe r0 hat keine IK und bleibt leer. Station: 16 nutzbare Plätze (12 voll, 4 halb). Oberkanten der Steinkörper je Lage: 124 / 245 / 366 / 487 mm über dem Boden.

### A.2 Stops und Dock

Der Kartenrahmen von floor_plan.md hat seinen Ursprung an der äußeren Anfangsecke der Leiste von A; x läuft entlang A, y zeigt zu ARES. Es gilt y_Karte = y_Wand + 60 mm. Kurs 0° heißt: ARES-x entlang A, ARES blickt auf B.

| Stop | Schenkel | Steine | base_link Wand (x, y) [mm] | Karte (x, y) [mm] | Kurs | Boards | Anfahrt |
|---|---|---|---|---|---|---|---|
| 0 | A | 24 | (160,0; 580,0) | (160; 640) | 0° | W0, W1 | Startmarke |
| 1 | B | 22 | (200,0; 602,7) | (200; 662,7) | 0° | W2, W4 | Route 90 mm: dx −20 / dy +22,7, dann dx +60 |
| 2 | B | 8 | (200,0; 862,7) | (200; 922,7) | 0° | W2, W4 | Route 327 mm: dx −60 / dy +260, dann dx +60 |
| 3 | C | 22 | (137,3; 822,7) | (137,3; 882,7) | 0° | W5, W6 | Route 189 mm: dx −122,7 / dy −40, dann dx +60 |
| Dock | – | – | (−1100; 840) | (−1100; 900) | 90° | S0, S1 | Stationsrouten 1,26–1,37 m je Richtung, je 1 Drehung um 90° |

Bei Stop-Routen endet die letzte Teilfahrt 40 mm vor dem Stop; den Rest fährt die Regelung nach der Messung. Stationsroute von Stop 0: dx −1260 / dy +200, Drehung +90°, dx +60 (Dock); zurück: dx −160 / dy −600, Drehung −90°, dx +600 / dy −100, dx +60 (davon 20 mm, 40 mm vor dem Stop).

### A.3 Startmarke Stop 0 (Kartenrahmen; Fahrzeugumriss nach [ares] length 1120 / width 600)

- base_link bei (160; 640). Umriss x −400 … 720, y 340 … 940.
- Rechte Seite: 220 mm von der ARES-seitigen Leistenkante von A (y 120), 110 mm von den Board-Platten (y 230).
- Vorderkante: 260 mm vor der ARES-seitigen Fläche von B (x 980), 150 mm vor den Platten von B (x 870).

### A.4 Boards ([[targets]] in station.toml; = targets/README.md §4)

| Board | IDs | Schenkel | Block k (Mitte u) | genutzt an |
|---|---|---|---|---|
| W0 | 30–39 | A | 0 (100 mm) | Stop 0 |
| W1 | 40–49 | A | 3 (700 mm) | Stop 0 |
| W2 | 50–59 | B | 1 (300 mm) | Stop 1, 2 |
| W3 | 60–69 | B | 3 (700 mm) | liegt aus, vom Job nicht genutzt |
| W4 | 70–79 | B | 5 (1100 mm) | Stop 1, 2 |
| W5 | 80–89 | C | 1 (300 mm) | Stop 3 |
| W6 | 90–99 | C | 4 (900 mm) | Stop 3 |
| W7 | 100–109 | – | Reserve | – |
| S0 / S1 | 200–209 / 210–219 | Station | Fenster bei Reihenmitte −500 / +500 mm | Dock |

### A.5 Magazin (ARES-Rahmen)

Aufbau:
- Reihe r0 liegt bei x −300 mm (653,6 mm hinter der UR-Achse), r1 bei x −80 mm (433,6 mm).
- Spalten y0 / y1 / y2 liegen bei y −205 / 0 / +205 mm.
- Halbsteinplatz r1y0: a bei y −255, b bei y −155 mm.
- Die Steinachse liegt entlang ARES-y.
- TCP-Höhe von Lage 1 / 2 / 3: 473,6 / 594,6 / 715,6 mm, bei holder_z 20.
- Lage 1 liegt unten.

Startfüllung in Entnahmereihenfolge; die Lagen darunter sind jeweils belegt:

| # | Platz | Typ | Stein |
|---|---|---|---|
| 1 | r1y2l3 | voll | Ac0i0 |
| 2 | r1y0al1 | halb | Ac1i0h |
| 3 | r1y1l3 | voll | Ac0i1 |
| 4 | r0y0l2 | voll | Ac1i1 |
| 5 | r0y1l2 | voll | Ac2i0 |
| 6 | r1y0bl1 | halb | Ac3i0h |
| 7 | r0y2l2 | voll | Ac0i2 |
| 8 | r1y2l2 | voll | Ac1i2 |
| 9 | r1y1l2 | voll | Ac2i1 |
| 10 | r0y0l1 | voll | Ac3i1 |
| 11 | r0y1l1 | voll | Ac0i3 |
| 12 | r0y2l1 | voll | Ac1i3 |
| 13 | r1y1l1 | voll | Ac2i2 |
| 14 | r1y2l1 | voll | Ac3i2 |

Damit sind belegt: r1y2 Lagen 1–3, r1y1 Lagen 1–3, r0y0/r0y1/r0y2 Lagen 1–2, r1y0 a/b Lage 1.

### A.6 Station (Stationsrahmen; Reihe bei y 210 mm, Steinachse entlang x)

s00–s05 bei x 225 / 430 / 635 / 840 / 1045 / 1250 mm, je 2 Vollsteine übereinander. h00 / h01 bei x 70 / 1405 mm, je 2 Halbsteine übereinander. Zusammen 12 voll und 4 halb. Beim Umladen wird die obere Lage (l2) zuerst genommen.

### A.7 Steinfolge Stop 0 und Beginn Stop 1

Bezeichnung: `A c<Lage> i<Index>`, `h` = Halbstein. u ist die Mitte entlang des Schenkels, "top" die Oberkante des Körpers.

| # | Stein | Typ | u [mm] | top [mm] | Magazinplatz |
|---|---|---|---|---|---|
| 1 | Ac0i0 | voll | 100 | 124 | r1y2l3 |
| 2 | Ac1i0h | halb | 50 | 245 | r1y0al1 |
| 3 | Ac0i1 | voll | 300 | 124 | r1y1l3 |
| 4 | Ac1i1 | voll | 200 | 245 | r0y0l2 |
| 5 | Ac2i0 | voll | 100 | 366 | r0y1l2 |
| 6 | Ac3i0h | halb | 50 | 487 | r1y0bl1 |
| 7 | Ac0i2 | voll | 500 | 124 | r0y2l2 |
| 8 | Ac1i2 | voll | 400 | 245 | r1y2l2 |
| 9 | Ac2i1 | voll | 300 | 366 | r1y1l2 |
| 10 | Ac3i1 | voll | 200 | 487 | r0y0l1 |
| 11 | Ac0i3 | voll | 700 | 124 | r0y1l1 |
| 12 | Ac1i3 | voll | 600 | 245 | r0y2l1 |
| 13 | Ac2i2 | voll | 500 | 366 | r1y1l1 |
| 14 | Ac3i2 | voll | 400 | 487 | r1y2l1 |
| – | Reload 1: 16 Steine (12 voll, 4 halb) | | | | |
| 15 | Ac0i4 | voll | 900 | 124 | r1y2l3 |
| 16 | Ac1i4 | voll | 800 | 245 | r1y1l3 |
| 17 | Ac2i3 | voll | 700 | 366 | r0y0l2 |
| 18 | Ac3i3 | voll | 600 | 487 | r0y1l2 |
| 19 | Ac0i5h | halb | 1050 | 124 | r1y0al2 |
| 20 | Ac1i5 | voll | 1000 | 245 | r0y2l2 |
| 21 | Ac2i4 | voll | 900 | 366 | r1y2l2 |
| 22 | Ac3i4 | voll | 800 | 487 | r1y1l2 |
| 23 | Ac2i5h | halb | 1050 | 366 | r1y0bl2 |
| 24 | Ac3i5 | voll | 1000 | 487 | r0y0l1 |
| Stop 1 | Bc0i0, Bc1i0h, Bc0i1, Bc1i1, Bc2i0, Bc3i0h | | 100, 50, 300, 200, 100, 50 | 124, 245, 124, 245, 366, 487 | r0y1l1, r1y0al1, r0y2l1, r1y1l1, r1y2l1, r1y0bl1 |

---

## Anhang B – Befehle

| Zweck | Befehl |
|---|---|
| Kamera finden und Testbilder | `py.exe tools/cam_check.py list`, `py.exe tools/cam_check.py grab -n 5`, `py.exe tools/cam_check.py live` |
| UR-Zustand, Bremsen, Werkzeugspannung, Pose, Greifer | `py.exe tools/ur_check.py info`, `… ready`, `… tool-voltage 24`, `… pose`, `… grip open` / `grip close` |
| IK-Prüfung PolyScope 3.3 (ohne Bewegung) | `py.exe tools/calib_handeye.py plan --check` |
| ARES lesen und fahren (HMI mit `--ares` offen) | `py.exe tools/ares_check.py status`, `py.exe tools/ares_check.py move --dy 100 --yes` |
| HMI SIM / REAL | `py.exe -m hmi --job data/jobs/nominal_C_robodk.json [--twin]`, `py.exe -m hmi --ares --job data/jobs/nominal_C_robodk.json [--twin]` |
| Kamera-Montagecheck | `py.exe tools/measure_target.py mount --boards calib --report data/mount_check/mount_<Datum>_ares.json` |
| Hand-Auge | `py.exe tools/calib_handeye.py orbit --dataset data/he_<Datum>_ares1`, `py.exe tools/handeye_solve.py <Datensatz> --check calib/handeye.json`, `py.exe tools/handeye_solve.py <Datensatz> --out calib/handeye.json --force` |
| Boards und Einschwingzeit | `py.exe tools/measure_target.py repeat --boards W0 --n 10 [--settle-s 0,0.5,1,2 --move-mm 20] --report <Datei>` |
| Freeze | `py.exe -m pytest -q tests`, `py.exe robodk/simulate.py --animate`, `py.exe tools/run_job.py data/jobs/nominal_C_robodk.json --dry-run`, `py.exe tools/make_stone_list.py --job data/jobs/nominal_C_robodk.json` |
| Notlösung nach HMI-Absturz | `py.exe tools/run_job.py data/jobs/nominal_C_robodk.json --real --step --stops k: (--resume-log <Ordner> \| --stop-untouched)` |

---

## Anhang C – Lücken in Software und Preflight, gefunden beim Schreiben (Plan D2)

Stand 2026-10-08 (`main` 7ab0be8, Plan D9).

1. **C1 (offen, mit S1):** Der REAL-Start in der HMI fragt nicht ab, ob die Backen leer sind, das Magazin wie die Liste belegt und die Station voll ist. Er zeigt auch die Start- bzw. Neustart-Belegung des Magazins nicht. `tools/run_job.py --real` kann beides seit 22340be. Bis das in der HMI ist, gilt die Checkliste aus T5b.
2. **C2 (offen, mit S1):** Für einen Start bei Stop k > 0 hat die HMI kein Gegenstück zu `--stop-untouched` oder `--resume-log`. Nach dem Merge setzt der Sequencer die Stops < k als gebaut voraus, der Bediener sieht die Füllung aber nicht. Einen teilweise gebauten Stop setzt man nach einem HMI-Neustart nur über die CLI fort.
3. **C3 (offen, mit S1):** In `hmi/core/rigs.py` fehlt `approach_mm=job.approach_mm` beim `MotionGuard` (Plan D6).
4. **C4 (offen, mit S1):** `hmi/core/preflight.py` sperrt mit `IK_GUARD_33` jede Steuerung mit PolyScope 3.3 pauschal. Seit `main` 8894751 gibt es `[ur] ik_check`; die Sperre muss beim Merge durch dessen Prüfung ersetzt werden, sonst bleibt Start gesperrt.
5. **C5 (erledigt, 7d9459f):** `preflight_real` blockiert bei PLACEHOLDERn, die außerhalb der Kameraregelung wirken: [ur5] mount_z / mount_rz, [deck] holder_z, [ur] payload_cog_mm, [boards.ref] square_mm, [camera] settle_s.
6. **C6 (offen):** Die HMI hat keinen Weg für eine `AresAds`-Testfahrt im eigenen Prozess ohne Lauf; HMI_DESIGN §14.1 wünscht sie am aufgebockten ARES. Ersatz in diesem Plan ist die absichtliche Korrekturfahrt in T5b. Vorschlag: ein Testfahrt-Knopf im Mauer-Tab (nur REAL, nur ohne Lauf, mit Bestätigung).
7. **C7 (erledigt, fce214f):** `config_sha256` und der RoboDK-Stempel lassen die `[hmi*]`-Tabellen aus; HMI-Einstellungen machen Jobs und Stempel nicht mehr ungültig.
8. **C8 (offen):** Ein neuer Lauf ab Stop k nimmt ARES an der Startmarke von k an. Die HMI warnt nicht, wenn die PLC-Odometrie zeigt, dass ARES noch am vorigen Stop steht.
9. **C9 (erledigt, c10d72a):** veraltete Texte in targets/README.md §4, in den Hilfetexten von `calib_handeye.py`, `measure_target.py` und `ur_check.py` und in docs/CAMERA_SETUP.md §10. Die Übersichtszeile oben in targets/README.md nennt für W0–W7 noch "L layout".

---

## Anhang D – Verhalten der HMI nach dem Review (2026-10-08)

Ein Review der HMI (`hmi-integration`) hat 23 Befunde ergeben; alle wurden korrigiert (docs/HMI_DESIGN.md §17). Für die Bedienung im Test wichtig:

- **HALT sperrt REAL den UR und ARES** (URLink / `AresAds`), bis **Start** oder **Resume** gedrückt wird. Eine Bewegung, die bei HALT gerade geplant wurde, wird nicht mehr gesendet; der Lauf endet dann "aborted". Eine laufende Bewegung wird gestoppt; der Lauf endet "error". HALT steht im Run-Log (`halt`, `halt_result`).
- **HALT, Pause und Abort während der Prüfungen von Start oder Resume** (REAL etwa 0,3–1 s) halten den Lauf vor der ersten Bewegung an. Früher gingen sie verloren.
- **REAL Start prüft den Preflight unmittelbar vor der ersten Bewegung** noch einmal und wartet, solange ARES fährt.
- **Pause kurz vor dem Laufende** lässt den Zustand nicht mehr in "pausing" hängen.
- **Jaws empty** setzt auch die Nutzlast und den gehaltenen Stein des UR-Backends zurück.
- **Confirm pose** gibt es nur für eine Odometrie-Schätzung ("unverified"); eine unbekannte Lage braucht **Set pose**.
- **Schließen** wird auch während einer Relativfahrt verweigert; beim Schließen endet zuerst der Heartbeat.
- **Rotes Banner**, wenn Leertaste / Esc die HMI nicht erreichen; Fenster-öffnende Knöpfe sind während eines Laufs gesperrt.
- **Mit `--ares`** zeigt die HMI die erste ADS-Verbindung zuverlässig an (vorher konnte sie "Disconnected" zeigen, obwohl der Heartbeat lief).
- **RTDE-Ausfall** während einer Roboterbewegung: nach 5 s Abbruch und "error" statt bis zu 180 s Warten.
- **Zählweise:** Stops ab 0, Teilfahrten ab 1.
