# Analyse Lauf 2026-10-16_182116 und neue Blickbewegungen des UR5 (2026-10-18)

Quelle: `data/runs/2026-10-16_182116_hmi_real/run.jsonl` (Tour C, ohne Steine), verglichen mit 132348 und 163318
vom selben Tag. Tour-Report: `py.exe tools/tour_report.py data/runs/2026-10-16_182116_hmi_real`. Auswerte-Skripte
der Analyse lagen im Scratchpad (nicht versioniert); alle Zahlen unten stammen aus den drei Lauf-Logs.

## 1. Sind die Korrekturen grösser geworden?

| Lauf | Dauer | ARES-Fahrten | Korrekturen | Korrektur-Translationen | Korrektur-Drehungen |
|---|---|---|---|---|---|
| 132348 | 22.5 min | 50 | 8 | ~30–56 mm je Stopp (40 mm Standoff) | 0.6–1.2° (5×) |
| 163318 | 14.6 min | 39 | 5 | 58, 47, 10 mm (+2 leere) | 2.0°, 1.4° |
| 182116 | 14.1 min | 36 | 6 | 3, **27**, 5, 10, 3, **32** mm | **2.5°**, 1.1°, 1.7° |

Nicht generell: die Translationen sind in Summe kleiner als in 163318 (dort 58/47 mm durch die inzwischen
behobene Stations-Schätzung). Grösser geworden sind zwei Dinge:

1. **Ankunft nach Stopp→Stopp-Fahrten**: Stopp 1→2 kam mit 31.7 mm / 1.9° an (163318: 8.5 mm / 0.35°,
   132348: 18.8 mm / 0.25°). Die Fahrt ist eine 558 mm Seitwärtsfahrt; 1.9° Gierfehler am Anfang einer 558-mm-Fahrt
   erklären allein ~18 mm des Querfehlers von 27 mm.
2. **Drehkorrekturen**: drei statt zwei, und sie sind eine Folge der Seitwärts-Korrekturen (siehe 2.1).

## 2. Schlupf oder systematischer Fehler?

Beides – mit klarer Gewichtung. Grundlage: jede Kamera-Messung gegen die kommandierten Fahrten seit der letzten
Messung in demselben Bezugssystem (Wand→Wand, Station-Rundfahrten Wand→Station→Wand).

### 2.1 Systematisch: Seitwärtsfahrten verdrehen ARES

Die SPS fährt eine Translation mit ω = 0 und **regelt die Richtung nicht nach** (MA `FB_RelMove.st`:
`fOmega_degs := 0.0`, `fHeadErr_deg` wird nur gemeldet). Was ARES während einer Translation verdreht, bleibt.

- In **15 von 18** Intervallen mit Seitwärtsfahrt (alle drei Läufe + Kurztests) dreht ARES entgegen dem
  Vorzeichen von dy (ARES-Frame dieses Repos: +y → −θ). Zufällig wäre das mit p ≈ 0.4 %.
- Mittlere Grösse ~0.55° pro Seitwärtsfahrt, **unabhängig von der Länge**: 27 mm → +2.16°, 10 mm → −0.73°,
  45 mm → +0.94°, 558 mm → −0.87° / +1.90°. Ein Effekt am Anfang der Fahrt (Lenkausrichtung / Losfahren),
  kein Drift pro Meter.
- Vorwärts-Korrekturen drehen praktisch nicht: 132348 +51 mm → −0.01°, +50 mm → +0.08°, +28 mm → +0.07°.

**Darum mehr Drehkorrekturen seit Standoff 0 (16.10.)**: Mit 40 mm Standoff waren die Korrekturen Vorwärtsfahrten
(+50 mm). Seit Standoff 0 sind sie fast reine Seitwärtsfahrten (dx ≈ 1 mm, dy 10–27 mm) – und die drehen ARES über
`stop_tol_deg` 1.0° hinaus. Beispiel Stopp 0 nach der Rückkehr: Messung 0.34°, Korrektur 27 mm seitwärts → 2.50°,
zweite Korrektur −2.5°. Stopp 1: −0.34° → 10 mm seitwärts → −1.07° → zweite Korrektur.

### 2.2 Systematisch: Stations-Rundfahrten enden zu kurz

Wand → Station → Wand (6–8 Fahrten, ~4 m, 2 × 90°): in **12 von 12** Rundfahrten endet ARES in x zu kurz
(−3 … −26 mm, Mittel −15 mm); der y-Fehler hat pro Stopp jedes Mal dasselbe Vorzeichen (Stopp 1: +25 … +55 mm,
Stopp 2: −27 … −51 mm).

### 2.3 Aber: der grösste Teil ist zufällig

Ein Fehlermodell (Skalierung x/y, Krabbenwinkel, Gieren pro Seitwärtsfahrt / pro Meter, Drehskalierung), gefittet
auf alle Kamera-Messungen der drei Läufe (Stationspose pro Lauf frei) und **pro Lauf ausgelassen validiert**:

| Modell | Position (rms) | Richtung (rms) |
|---|---|---|
| ohne Modell | 11.2 mm | 0.70° |
| Skalierung (sx +0.3 %, sy −0.7 %) | 10.2 mm | 0.71° |
| Skalierung + Krabbe + Gieren seitwärts | 10.1 mm | 0.69° |
| alles | 9.6 mm | 0.62° |

Die systematischen Anteile erklären nur ~10–15 %. Der Rest (≈ 10 mm / 0.6° pro Fahrt) streut zufällig: Schlupf,
Boden, Lenkrollen. Eine Vorsteuerung/Kalibrierung auf ARES-Seite bringt deshalb wenig; die Kamera-Korrektur bleibt
nötig.

### 2.4 Was offen ist – und neu geloggt wird

`run.jsonl` enthielt bisher die SPS-Odometrie der Fahrten nicht. Damit lässt sich nicht trennen, ob die Odometrie
das Gieren gesehen hat (Rad-Wegdifferenz, nur nicht nachgeregelt) oder ob es Schlupf ist (Odometrie blind). Ab jetzt
schreibt der Sequencer pro Fahrt `odom` (dx, dy, dθ im ARES-Frame) und `lat_err_mm` / `head_err_deg` /
`final_err_*` der SPS ins Log; `tools/tour_report.py` zeigt sie in der Fahrten-Tabelle.

### 2.5 Mögliche Gegenmassnahmen (nicht umgesetzt – Entscheid Samuel)

- **Gieren bei Seitwärtsfahrt**: Heading in der SPS während Translationen nachregeln (MA-Repo, `FB_RelMove`), oder
  im Sequencer die erwartete Drehung (~0.55° gegen dy) vorsteuern. Streuung 0.2–2.2° bleibt.
- **`stop_tol_deg` 1.0° lockern**, solange jeder Stein erreichbar ist (die Steine werden ohnehin im gemessenen
  Rahmen gesetzt): spart die zweite (Dreh-)Korrektur in 2 von 4 Stopps dieses Laufs.
- Wieder einen kleinen Standoff (z. B. 20 mm), damit Korrekturen eher Vorwärtsfahrten sind.

## 3. UR5-Bewegungen zu den Codes und weg von den Codes

### 3.1 Befund im Lauf

Die Blick- und Parkbewegungen des UR brauchten **577 s von 847 s** der Tour (68 %). Teuer waren:

- `stop1-W5` 18.6–19.4 s, `station-S1` 19.4–21.9 s, `stop0-W5` nach Stationsfahrten 14–18 s (statt 7 s bei
  Ankunft): Umwege mit 4–6 Zwischenpunkten, an **jedem** hielt der Arm an (movej ohne Überschleifen).
- War der direkte Weg frei (leeres Magazin bei Ankunft), schwenkte der Arm tief über das Magazin zum zweiten Code;
  mit vollem Magazin wurde daraus der Umweg – je nach Zustand ein anderer Weg.

### 3.2 Neues Schema (mauer/motionguard.py, `plan(..., look=True)`)

Park → Code → Code → Park:

1. **Weg vom Code nach oben** in 100-mm-Schritten (gleiche Orientierung) auf 150–550 mm über den höheren Code
   (niedrigste freie Höhe) – senkrecht, oder wo ein Code am Fuss der hohen Mauer / eines Stationsstapels liegt
   (die Backen träfen senkrecht darüber die oberen Lagen) um 27° bzw. 45° zur UR-Basis hin geneigt,
2. **Schwenk** zur Position über dem nächsten Code – ein Gelenkzug, oder mit Wegpunkten alle 30° Basisdrehung, die
   den TCP mittig 150/300 mm zur UR-Basis einziehen (Abstand zur Mauer / zu den Stationsstapeln),
3. **hinunter auf den Code**, ebenso senkrecht oder leicht geneigt.

Unter allen freien Varianten gewinnt die mit dem kürzesten Gelenkweg. Alle Zwischenpunkte werden **überschliffen**
(URScript `movej(..., r=)`, Radius ≤ 100 mm und ≤ 45 % der Nachbarstrecken, damit sich Blends nie überlappen – der
UR würde sonst einen Wegpunkt überspringen): der Arm hält nur noch am Code (Bild) und in der Parkpose. Die
überschliffenen Ecken prüft der Guard mit (Bezier-Kurve durch die Ecke, `motionguard.blend_curves`). Findet sich
keine Variante, kommen der hohe Bogen (senkrechte Enden bleiben erhalten) und die bisherigen Umwege, überschliffen wo
die Ecken frei sind. Park nach einem Code fährt ebenfalls zuerst nach oben. Die IK-Prüfung des Codes im URScript
steht jetzt vor der ersten Bewegung des Blocks.

Offline (`tools/make_job.py` `look_moves` / `plan_look_vias`) werden alle Blickbewegungen im ungünstigsten Zustand
(Magazin voll, Mauer bis Ende des Stopps gebaut, Station voll) geplant und im Job abgelegt; zur Laufzeit prüft der
Guard die Job-Wege zuerst und plant nur neu, wenn sie (nach dem Nachzielen auf die gemessene ARES-Pose) blockiert sind.

### 3.3 Wirkung (Guard-Modell, ungünstigster Zustand, je ein Durchgang)

Bewegungszeit nach `motionguard.move_time_s` bei `[ur] v_joint` / `a_joint` 0.5 (überschliffen: untere Schranke,
bisher: Halt an jedem Zwischenpunkt). Ohne Einschwingzeit `[camera] settle_s` 1 s und Bildaufnahme.

| Ort | Park→1. Code / 1.→2. Code / 2. Code→Park bisher | neu |
|---|---|---|
| Stopp 0 (W2, W5) | 10.3 / 13.3 / 5.5 s | 6.1 / 9.2 / 4.3 s |
| Stopp 1 (W2, W5) | 7.6 / 16.1 / 9.6 s | 5.1 / 9.0 / 5.0 s |
| Stopp 2 (W0, W1) | 12.1 / 13.8 / 6.1 s | 6.3 / 5.0 / 5.0 s |
| Stopp 3 (W6, W7) | 9.0 / 14.3 / 11.0 s | 6.1 / 8.3 / 7.5 s |
| Station (S0, S1) | 8.6 / 16.8 / 9.0 s | 5.6 / 8.1 / 5.4 s |
| **Summe** | **163 s** | **96 s** |

Planungszeit zur Laufzeit 0.1–1.7 s pro Bewegung (bisher bis 7 s); mit den Job-Wegen meist eine einzige Prüfung.

## 4. Verifikation (2026-10-18)

- **Guard-Modell**: alle 15 Blickbewegungen (4 Stopps + Station) frei im ungünstigsten Zustand, jede verlässt und
  erreicht ihren Code von oben (senkrecht oder ≤ 45° zur Basis geneigt), alle Blends überlappungsfrei
  (`tests/test_motionguard.py`, neue Tests inkl. URScript-Ausgabe `movej(..., r=)` und Park-nach-oben).
- **SimWorld** (realistische ARES-Fehler, Guard aktiv, gleicher Seed alt/neu):

  | Lauf | Blick + Park, geschätzte Bewegungszeit | ARES-Fahrten / Korrekturen | Ergebnis |
  |---|---|---|---|
  | Tour Seed 1 | 428 s → 304 s (−29 %) | 35 / 4 | done, 0 Verletzungen |
  | Tour Seed 2 | 394 s → 283 s (−28 %) | 34 / 3 | done, 0 Verletzungen |
  | voll Seed 1 / 2 (neu) | – | 35 / 4, 34 / 3 | 67/67 gesetzt und sitzend, 0 Verletzungen |

- **RoboDK** (`py.exe robodk/simulate.py`, Stand d4c8957): 67/67 Steine, 53/53 Stationstransfers, 11 Routen ohne
  Kollision, **48/48 Blickbewegungen frei** (jeder Stopp bei Ankunft, nach jeder Stationsfahrt und fertig gebaut;
  Station bei jeder Fahrt), überschliffene Ecken eingeschlossen. Job gestempelt
  (`data/jobs/nominal_C_robodk.json`); `stamp_problems`, `route_problems`, `preflight_real` leer.
- **Testsuite**: grün bis auf `tests/test_make_guides.py::test_pieces_do_not_overlap_in_the_room_and_dovetails_interlock`,
  das schon auf 3392ef4 fehlschlägt (Führungsteile, nicht angefasst). Drei Routen-Tests und zwei Sequencer-Tests
  erwarteten noch die getrennte Anfahrt von vor `merge_approach` – angepasst.

Offen für den echten Lauf: Die Überschleif-Radien (`BLEND_MM` 100 mm, ASSUMPTION) und das Verhalten des CB3 beim
Überschleifen langsam beobachten (HMI-Geschwindigkeit reduziert); die Odometrie-Felder im neuen Log mit
`tools/tour_report.py` auswerten (Schlupf oder ungeregelte Drehung).

## 5. Richtungstoleranz gelockert: `stop_tol_deg` 1.0° → 2.0° (2026-10-18)

Samuel: „die Toleranz kann man noch ein bisschen lockern solange alle Steine erreichbar bleiben“.

- **Reichweite**: IK-Prüfung aller Steine des C-Jobs mit ARES um ±1.0 … 3.0° gedreht, am Stopp und ±20 mm daneben: bis
  3° fällt kein Stein heraus, der nicht schon bei ±20 mm / 1.0° fehlt (die drei Halbsteine am Rand: Bc3i0h, Bc3i9h,
  Ac3i0h). Die Laufzeitprüfung korrigiert dort ohnehin.
- **Nachgespielt an den echten Läufen**: 2.0° spart eine Korrektur (163318 Stopp 1, 1.36°), 2.5° zusätzlich die von
  182116 Stopp 0 (2.50°, genau an der Grenze).
- **Erreichbar heisst jetzt auch kollisionsfrei**: Die SimWorld mit 2.0° zeigte, dass die reine IK-Prüfung nicht
  reicht. Seed 2 kam an Stopp 2 mit 19.7 mm / 1.01° an, alle Steine hatten IK, aber der Guard verweigerte das Setzen
  des Ecksteins Ac0i0 (Unterarm 1 mm zu nah an einem Magazinstein) und der Lauf stoppte. Das wäre auch mit 1.0° bei
  0.99° möglich gewesen. Die Reichweitenprüfung des Sequencers (`_unreachable`) fragt deshalb jetzt zusätzlich den
  Motion-Guard nach der Anfahrpose jedes noch zu setzenden Steins (`MotionGuard.target_problems`, dasselbe Ziel, das
  der Guard vor dem Setzen prüft; Mauer mit den Steinen davor, Magazin ohne deren geplante Plätze). Ein Stein mit
  Kollision gilt als nicht erreichbar → Korrektur zum Ziel, von dem aus alle frei sind.
- **SimWorld** (realistisch, Guard, volle Läufe mit Steinen, Seeds 1–4, beide Toleranzen): 8/8 fertig, 67/67 sitzend,
  0 Verletzungen. Korrekturen nur wegen der Richtung: 2 (1.0°) → 0 (2.0°); Positions- und Kollisionskorrekturen
  streuen zufällig mit dem geänderten Fahrverlauf (Summe 12 / 12). Die Kollisionsprüfung meldete nur Ac0i0, und
  nur wenn ARES 20–34 mm neben Stopp 2 stand.
- **RoboDK** (Stand 6a49488, 2026-10-18 17:34): 67/67 Steine, 53/53 Stationstransfers, 11 Routen ohne Kollision,
  48/48 Blickbewegungen frei; Job gestempelt, `stamp_problems` / `route_problems` / `preflight_real` leer.
- **Testsuite**: grün bis auf das bekannte `test_make_guides` (Führungsteile, schon vor diesen Änderungen rot).
