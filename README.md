# Robot Dynamic Diffusion (RDD)
Ziel des Projektes ist es, die räumliche Wahrnehmung von Musik während Aufführungen so auszunutzen, dass ein komplett neues Klangerlebnis entsteht. Die räumliche Akustik ist hierbei von der Position der Klangquelle abhängig. Je nachdem auf welcher Position in einem Raum die Klangquelle steht, werden die Schallwellen unterschiedlich von den Wänden zurückgeworfen. Dies resultiert in einer sich ändernden Akustik. Als Beispiel kann hier ein einfacher Lautsprecher genannt werden. Wird der Lautsprecher in eine Ecke des Raumes positioniert, werden sich die Schallwellen anders ausbreiten als ein Lautsprecher, der in der Mitte des Raumes aufgebaut wurde. Ein Unterschied in der Wahrnehmung des Klangs ist somit die Folge. Dieses Phänomen wird im Projekt Moving Speakers ausgenutzt. Es werden Lautsprecher auf einem Roboter montiert. Dieser wird dann über eine entsprechende Ansteuerung im Raum bewegt. Wird über die Lautsprecher Musik abgespielt, kann die sich ändernde Akustik wahrgenommen werden.

## Rahmen des Projekts
Die grundsätzliche Idee der Ausnutzung des räumlichen Klangs mit Hilfe geeigneter mobiler Robotik, wurde von der Band **Mouse on Mars** entworfen. Durch Zusammenarbeit der Projektklasse [**Dynamische Akustische Forschung**](https://adbk-nuernberg.de/studium/kuenstlerische-klassen/interaktive-medien-dynamische-akustische-forschung/) der **Akademie der bildenden Künste** in Nürnberg, der **Technischen Hochschule Georg-Simon-Ohm in Nürnberg** und dem Unternehmen [**Evocortex**](https://evocortex.org/), wird die Realisierung des Projekts beschrieben.\
Die Mitarbeiter der Akademie der bildenden Künste (AdbK) kümmern sich hierbei um die Realisierung einer App, mit deren Hilfe der Roboter in seiner Bewegung und in seiner Position gesteuert werden soll. Außerdem hat die AdbK die Aufgabe den Aufbau der Lautsprecheranlage und der dazugehörigen Elektronik auf der Roboterplattform zu realisieren.\
Die Roboterplattform wird von dem Unternehmen Evocortex bereitgestellt. Da der Roboter wie ein Instrument in Auftritten verwendet werden soll, wird die App so programmiert, dass sie über das sogenannte MIDI-Protokoll den Roboter steuern soll.\
MIDI ist ein Industriestandard, der es elektronischen Musikinstrumenten wie Synthesizern oder elektronischen Klavieren ermöglicht, unabhängig des Herstellers, untereinander zu kommunizieren. Diese Kommunikation und Anbindung der elektronischen Instrumente sind bei Konzerten unerlässlich. Da der Roboter jedoch nicht in der Lage ist, MIDI-Kommandos zu verarbeiten, musste eine Schnittstelle realisiert werden, die die Anbindung des Roboters an die App und somit an ein MIDI-System ermöglicht.\
Für diese MIDI-Schnittstelle zwischen App und Roboterplattform ist die Projektgruppe der TH Nürnberg, bestehend aus den Studenten Gabriel Kaufmann und Jano Hanzlik, verantwortlich.\
Zwei weitere Projektgruppen der TH kümmern sich um die Navigation und um das gefahrenlose Fahren des Roboters.\
Die Schnittstelle wurde innerhalb des Robot Operating Systems (ROS) in 7 verschiedenen Knoten programmiert.
<p align="center">
  <figure>
    <img src="/pictures/ProjektstrukturKomplett.jpg" border="51" title="Darstellung der Knotenstruktur">
  </figure>
</p>

Die App, die von der AdbK entwickelt wird (in Abbildung links), sendet hierbei MIDI-Kommandos an den MIDI Converter, eine JSON File an den Knoten WP Receiver und eine JSON File mit den Konfigurationsdaten der MIDI Steuerung an den Knoten Config Receiver. Die Schnittstelle, also die 7 Knoten, wie sie in der Abbildung zu sehen sind, werden auf der Recheneinheit des Roboters (oben rechts in der Skizze) gestartet.\
Die Aufgaben der einzelnen Knoten und die zwei Steuerungsmodi, mit denen der Roboter bewegt werden kann, werden nach einer kurzen Einführung in das MIDI-Protokoll näher erläutert.

### MIDI
MIDI ist - wie in der Einführung bereits erwähnt - ein Industriestandard, der es elektronischen Musikinstrumenten wie Synthesizern oder elektronischen Klavieren ermöglicht, unabhängig des Herstellers, untereinander zu kommunizieren. In einem MIDI-System werden hierbei MIDI-Kommandos zwischen den jeweiligen Einheiten hin- und hergeschickt.\
MIDI-Kommandos bestehen insgesamt aus 24 Bits. Davon sind 2 Bytes Datenbytes und 1 Byte ein Statusbyte. Die Abbildung zeigt hierbei eine beispielhafte MIDI-Nachricht.

![Darstelllung der drei Bytes eines MIDI-Kommandos (Beispiel)](/pictures/MIDIstruktur.PNG?raw=true "Darstelllung der drei Bytes eines MIDI-Kommandos (Beispiel)")

Das **Statusbyte** liefert grundsätzlich Informationen über den Message kind also über die Art der Nachricht und über den Channel also über den jeweiligen Kanal. Die Art der Nachricht gibt hierbei erstmal Auskunft, ob zum Besipiel eine Note gespielt wird, oder ob sie stumm ist. Hier gibt es etliche verschiedene Nachrichtenarten, welche aus der Dokumentation von [**MIDI**](https://www.midi.org/midi-articles/about-midi-part-3-midi-messages) entnommen werden können. Anhand der Art der Nachricht soll festgestellt werden, ob eine manuelle Steuerung des Roboters oder die Steuerung über Wegpunkte erfolgt. Bei der Manuellen Steuerung werden nochmals unter zwei verschiedenen Arten unterschieden. Hierbei wird festgestellt, wann der Roboter in eine bestimmte Richtung fahren soll und wann er diese Bewegung wieder unterbinden soll. MIDI erlaubt es 16 verschiedene Kanäle in einem MIDI-Setup aufzuspannen. Hierbei können nur Geräte miteinander kommunizieren, die sich in demselben Kanal befinden. Hierdurch könnte unter anderem noch der Einsatz weiterer Roboter ermöglicht werden.\
Die **Datenbytes** geben Auskunft über die Eigenschaften Pitch und Velocity einer MIDI-Nachricht. Die Eigenschaft Pitch gibt bei einem elektrischen Klavier normalerweise die Tonhöhe, also die jeweilige Taste, die angeschlagen wurde, an. Der Wert Velocity gibt in diesem Beispiel an, wie schnell die Taste angeschlagen wurde. Bei einer langsamen Betätigung der Taste, ist der Wert klein. Wird die Taste schnell betätigt, ist der Wert groß. Mit Hilfe dieser Werte kann dann die jeweilige Aktion des Roboters in den weiteren Knoten bestimmt werden.

## Steuerung
Die Steuerung unterteilt sich in zwei verschiedene Steuerungsmodi, welche je nach Bedarf verwendet werden können.\
Es gibt die **manuelle Steuerung**, hier wird der Roboter über Midi Signale direkt gesteuert, so kann man die Roboterplattform nach Wunsch geradeaus, rückwärts, seitlich nach rechts, seitlich nach links fahren oder sich um die eigene Achse rotieren lassen.\
Die Zweite Steuerung ist die sogenannte **Wegpunkt-Steuerung**. Hier fährt der Roboter vordefinierte Wegpunkte an. Jeder Wegpunkt besteht aus den x und y Koordinaten, dem Winkel zur relativen x-Achse (alpha) und dem Winkel zur relativen z-Achse (beta) um den Roboter im Anschluss exakt zu positionieren und auszurichten. Diese Informationen nehmen Bezug zu der im Vorfeld kartographierten Karte der Umgebung, welche der Steuerungssoftware vorliegt.

## Manuelle Steuerung
Um die Roboterplattform manuell über MIDI-Befehle steuern zu können, werden drei Knoten benötigt (**MIDI Converter, JSON Receiver, Manual Controller**).\
Diese werden im Folgenden beschrieben.

### JSON Receiver
Bevor man den Roboter manuell steuern kann, muss die Steuersoftware dem Roboter mitteilen, welches Signal, welchen Bewegungsbefehl ausführen soll. Für diesen Zweck erstellt die Benutzer GUI eine JSON Config Datei. Hier sind die jeweiligen Befehle mit den dazugehörigen MIDI-Informationen gelistet. Diese JSON-Datei sieht wie folgt aus:

![JSON Datei mit der Konfiguration für die MIDI-Befehle](/pictures/JSONConfig.PNG?raw=true "JSON Datei mit der Konfiguration für die MIDI-Befehle")

Die JSON-Datei, die die Konfiguration der manuellen MIDI-Befehle enthält wird nach dem Booten des Roboters (siehe Inbetriebnahme) über SFTP oder im laufenden Betrieb über TCP an den Roboter gesendet. Liegt die Datei vor, liest der Knoten die Daten aus und schreibt sie in die Nachricht **midiconfig.msg**, die sich wie folgt zusammensetzt:

![Inhalt der Message midiconfig.msg](/pictures/midiconfigmsg.PNG?raw=true "Inhalt der Message midiconfig.msg")

Der Knoten sendet die "midiconfig.msg" über die Topic "manualinstructions" an den Knoten **Manual Controller**.

### MIDI converter
Der Knoten „MIDI converter” ist für die Umwandlung der MIDI-Kommandos zuständig, die von der App gesendet werden.\
Der MIDI Converter empfängt die von der App gesendeten MIDI-Signale über ein Funkmodul und liest die Nachrichten aus. Hierbei werden die Werte **Message Kind, Channel, Pitch, Velocity** und der **Timestamp**, der Nachricht ausgelesen. Der Timestamp gibt zusätzlich zu den Eigenschaften der MIDI-Nachricht Auskunft über den Zeitstempel der jeweiligen Nachricht. Somit kann jede Nachricht chronologisch zugeordnet werden.\
Diese Werte werden in die custom message **manualcontrol.msg** gefüllt. Diese sieht wie folgt aus:

![Inhalt der Message manualcontrol.msg](/pictures/manualcontrolmsg.PNG?raw=true "Inhalt der Message manualcontrol.msg")

Der Knoten trifft außerdem eine Unterscheidung, ob der Befehl für die **manuelle Steuerung** oder für die **Wegpunkt-Steuerung** vorgesehen ist.\
Hierbei wird also der Message Kind verglichen. Ist der Message Kind **14**, dann wird die custom Message über die Topic **midiwaypoint** an den Knoten **WP Controller** gepublisht.\
Ist der Message Kind **8** oder **9**, dann wird die custom Message über die Topic **midimanual** an den Knoten **Manual Controller** gepublisht.\
Um zu garantieren, dass der Roboter nur fährt, solange er sich in Reichweite des Funkmoduls befindet, wird ebenfalls ein Alive Signal über MIDI gesendet.
Dieses Signal besteht aus einer MIDI Clock message und besitzt den Message Kind 120. Diese Nachricht wird ebenfalls über die Topic **midimanual** gepublisht.

### Manual controller
Der Knoten **manualcontroller** nutzt die von dem **jsonreceiver** gelieferten Konfigurationen und die vom „midiconverter“ gesendeten Befehle zu interpretieren und letztendlich auszuführen.\
Ist eine Konfigurationsdatei überreicht worden, werden diese in Form eines Dictionary lokal im Knoten gespeichert. Im Aufbau könnte dies so aussehen:

![Zuordnung Pitch zu der jewiligen Aktion des Roboters](/pictures/DictConfig.PNG?raw=true "Zuordnung Pitch zu der jewiligen Aktion des Roboters")

Über eine Switch-Case Abfrage des übergebenen Pitch-Wertes der MIDI-Nachricht, wird die jeweilige Aktion ermittelt.\
Über den Velocity-Wert und der jeweiligen Richtung wird dann eine **geometry.msg** beschrieben, welche auf der Topic **cmd_vel** gepublisht wird. Der Roboter bewegt sich.\
Um eine sichere Steuerung zu ermöglichen, bei der der Roboter nicht gegen ein Hindernis gefahren werden kann, wurde ein weiteres Projektteam der TH Nürnberg mit eingebunden.
Die Nachrichten, die sonst auf der Topic **cmd_vel** gepublisht werden würden, werden somit erst auf der spezifischen Topic für das sichere Fahren gepublisht. Der jeweilige Subscriber dieser Topic ermittelt dann das Gefahrpotenzial und leitet die Nachricht im Falle einer einwandfreien Fahrt über die Topic **cmd_vel** weiter, um die Bewegung des Roboters zu ermöglichen.

Ist der Roboter außer Reichweite des MIDI-Funkmoduls, erhält das System kein Alive-Signal mehr. Der Roboter detektiert dies und sendet eine **geometry.msg**, die die Plattform anhält.

## Wegpunktsteuerung
Die Steuerung des Roboters über vorgegebene Wegpunkte besteht aus **5 Knoten** und dem **Navigation Stack**, der von einem weiteren Projektteam der TH Nürnberg entwickelt wird.\
Die Kartografierung, die Lokalisierung sowie die Navigation wird von dem Projektteam für den Navigation Stack bearbeitet, weshalb dieser Teil hier nicht beschrieben wird.\
Bei den Knoten, die die Steuerung und die Schnittstelle zwischen der Steuerungssoftware und dem Roboter ermöglichen, handelt es sich um den **MIDI Converter**, auf den hier nicht weiter eingegangen wird, da er bereits unter der manuellen Steuerung beschrieben wurde, dem **Mapsender**, dem **Positionsender**, dem **JSONWPReceiver** und dem **WP Controller**.\
Diese Knoten werden im Folgenden erläutert.

### JSON Receiver WP
Dieser Knoten liest eine Wegpunktliste ein. Diese Liste wird ebenfalls zu Beginn der Inbetriebnahme über SFTP oder im laufenden Betrieb über eine TCP Verbindung lokal auf dem Roboter abgelegt.\
Die JSON-Datei, die die Wegpunkte enthält sieht wie folgt aus:

![JSON Datei mit der Wegpunktliste](/pictures/JSONWP.PNG?raw=true "JSON Datei mit der Wegpunktliste")

Die Wegpunkte werden von dem Knoten ausgelesen. Diese Informationen werden als Dictionary abgespeichert. Der Key ist die Wegpunktnummer und der Wert beinhaltet die Informationen des passenden Wegpunktes. Die Wegpunktliste wird überschrieben, sollte eine neue Datei abgelegt werden.\
Der Knoten fungiert als service Server und bietet den Service **getWaypoint.srv**. Dieser Service enthält folgende Informationen:

![Service Datei zum Abrufen eines Wegpunktes](/pictures/getWPsrv.PNG?raw=true "Service Datei zum Abrufen eines Wegpunktes")

Möchte nun ein Client Knoten die Beschreibung eines Wegpunktes, muss er diesen Service mit der gewünschten Wegpunktnummer aufrufen. Der Client Knoten ist in der Regel der Knoten **WPcontroller**.

### WP Controller
Der **wpcontroller** empfängt die Nachricht über die **midiwaypoint** Topic von dem **midiconverter**.\
Bei der Benutzung der Wegpunktsteuerung wird der MIDI Message Kind "Pitchbend" benutzt. Dieser besitzt insgesamt 2 Bytes, die den Pitchwert darstellen, indem Pitch und Velocity zusammengefügt werden. Dieser Pitchwert stellt im Prinzip eine Wegpunktnummer aus der Wegpunktliste dar. So weiß der Knoten, welche Wegpunktnummer als nächstes angefahren werden soll. Noch hat dieser Knoten jedoch nicht die Information, wo sich dieser Wegpunkt überhaupt befindet. Diese Information erhält der **WPController** über den **getwaypoint**-Service, den der **JSON WP Receiver**-Knoten aufspannt. Stellt der **WPController** nun also mit dem Pitchwert (=Wegpunktnummer) eine Anfrage bei dem **getwaypoint**-Service, so erhält er die weiteren Informationen, wie Position und Ausrichtung zu dem zugehörigen Wegpunkt (siehe **JSON WP Receiver**).\
Die Informationen über den jeweiligen Wegpunkt werden dann an den **Navigation Stack** weitergegeben, der - wie unter dem Rahmen des Projekts erwähnt - von einem weiteren Projektteam der TH entwickelt wird. 

### Mapsender
Um auf der Benutzersoftware Wegpunkte auswählen zu können, wird die Karte benötigt, die von dem Roboter vor der Nutzung der Wegpunktsteuerung aufgezeichnet werden muss.\
Der Knoten **mapsender** liest die aufgezeichnete Karte von dem Roboter in Form eines PMG aus. Dieses Bild wird dann über eine TCP Verbindung an die Benutzersoftware gesendet, um die Wegpunkteingabe zu ermöglichen.

### Positionsender
Der **Positionsender** liest alle 100 Millisekunden die Koordinaten der Roboterplattform aus um diese an die graphische Oberfläche via UDP zu senden.\
So ist es möglich die Position des Roboters Live in der Benutzersoftware anzuzeigen.

## Inbetriebnahme
Um den Roboter mit der beschriebenen MIDI-Schnittstelle steuern zu können, müssen einige Schritte beachtet werden. Auf diese Schritte wird im Folgenden eingegangen.

#### 1. Schritt: Hardware-Setup des Roboters
Die Batterie sowie die notwendige Peripherie (Router, Laserscanner, MIDI-Empfänger) müssen an die Schnittstellen des Roboters angeschlossen werden. 

#### 2. Schritt: Anschalten des Roboters
Der Roboter ist durch den Taster an der Seite des Gehäuses einzuschalten. Der Bootvorgang dauert einige Sekunden. 

#### 3. Schritt: Verbindung zum Netzwerk des Roboters herstellen
Nach dem Bootvorgang ist der Rechner, auf dem die Steuerung mit der Benutzeroberfläche erfolgt, mit dem WLAN-Netzwerk mit der SSID "EvoFE-2006001" zu verbinden.
Das WLAN-Passwort lautet "24229142".
Gegebenenfalls kann es einige Minuten dauern, bis der Roboter vollständig gebootet und das WLAN-Netzwerk aufgebaut ist. 

#### 4. Schritt: Aufbauen der SSH-Verbindung
Um den Roboter über den Rechner ansprechen zu können, ist eine SSH-Verbindung aufzubauen. 
Dies erfolgt über den folgenden Befehl auf der Kommandozeile:
```
ssh nvidia@10.42.0.1
```
Anschließend ist das Passwort "nvidia" einzugeben.
Dieses Passwort wird auch für die administratorischen Rechte verwendet (Nach jedem sudo Befehl).
Besteht die SSH-Verbindung, können mit Hilfe von [tmux](https://man7.org/linux/man-pages/man1/tmux.1.html) mehrere Instanzen vom Terminal geöffnet werden. 

#### 5. Schritt: Anpassen der Parameter in Configparameters.launch
In der Launchfile Configparameters.launch befinden sich alle notwendigen parameter, die der Roboter für die Schnittstellen benötigt. Die Parameter sind hierbei anzupassen.

#### 6. Schritt: Senden der MIDI-Konfigurationsdatei
Nun ist die JSON-Datei, die die Konfiguration für die MIDI-Befehle enthält über SFTP auf dem Roboter abzulegen, alternativ kann sie auch über TCP zum Roboter gesendet werden. Ist bereits eine aktuelle Konfigurationsdatei vorhanden, entfällt dieser Schritt.

#### 7. Schritt: Initialiserung des CAN-USB Adapters
Mit Ausführen des folgenden Befehls im Terminal, kann die Initialisierung des CAN-Busses durchgeführt werden, der die Kommunikation innerhalb des Roboters ermöglicht:
```
sudo ~/catkin-ws/src/evo_rd_platform/evo_rd__platform_example/scripts/can_init.sh
```

#### 8. Schritt: Motorcontroller des Roboters initialisieren und aktivieren
Um den Motorcontroller benutzen zu können, muss die zugehörige Launchfile ausgeführt werden.
Dies kann über den folgenden ROS-Befehl bewerkstelligt werden:
```
roslaunch evo_rd_platform_example evo_base_driver.launch
```
Bei einer Fehlermeldung ist das Launchfile erneut auszuführen. 

#### 9. Schritt: MIDI-Interface initialisieren
Um eine neue Terminalinstanz über [tmux](https://man7.org/linux/man-pages/man1/tmux.1.html) zu starten, ist zunächst der Befehlsmodus mit der Tastenkombination "Strg + b" zu öffnen, um anschließend über "c" eine neue Instanz zu öffnen.\
Dann kann in der neuen Instanz die Initialisierung des MIDI-Interfaces mit Hilfe der Ausführung des Launchfiles erfolgen:
```
roslaunch rdd_movingspeaker midiapi.launch
```
Nun kann die manuelle Steuerung benutzt werden.

#### 10. Schritt: Karte Aufzeichnen/Senden
Um die Wegpunktsteuerung (wird noch erarbeitet) zu nutzen, muss entweder die Karte bereits vorhanden sein, oder die Umgebung kartografiert werden.

#### 11. Schritt: JSON-WP Liste übertragen
Ist die Karte an der Bediensoftware angekommen, kann die Wegpunktliste generiert werden.\
Diese Liste ist dann entwerder über SFTP auf den Roboter abzulegen oder über TCP an den Roboter zu senden.\
Ist dies erfolgt, kann über die Bedienoberfläche die Wegpunktsteuerung benutzt werden.

**Um die Wegpunktsteuerung zu benutzen muss der Navigation Stack durch die mitarbeitende Projektgruppe ergänzt werden. 
Danach kann die Navigation und somit die Wegpunktsteuerung genutzt werden.**

#### 12. Schritt: Abschalten des Roboters
Um den Roboter zum Schluss wieder herunterzufahren, ist der folgende Befehl zu verwenden:
```
sudo shutdown -now
```
Ist der Roboter heruntergefahren, kann die Spannung durch das Drücken des Tasters an der Seite des Gehäuses abgeschalten werden.


## Fazit
Das Ziel der Schnittstelle zwischen der entwickelten App und der Roboterplattform wurde erreicht.\
Die manuelle Steuerung des Roboters funktioniert in so weit, dass der Roboter über ein MIDI-Gerät, wie ein elektronisches Klavier gesteuert werden kann. Ergebnisse über die letztendliche Klangqualität konnten zum Abschluss der Projektarbeit der MIDI-Schnittstelle noch nicht erfasst werden, da der Aufbau der Lautsprecheranlage noch nicht fertigstellt wurde. Mit Hilfe der Schnittstelle ist das Steuern des Roboters jedoch wie gewünscht möglich. Der Roboter kann somit wie ein elektronisches Musikinstrument in das Setup auf einer Aufführung mit eingebunden werden. Die öffentliche Prämiere des Roboters soll Mitte des Jahres 2021 stattfinden. Genauere Informationen hierüber sind noch nicht bekannt.\
Als Weiterentwicklung des Roboters ist denkbar, dass über eine künstliche Intelligenz die manuelle Steuerung des Roboters entfallen könnte. Der Roboter soll sich dann rein über abgespielte Musik intelligent zu dem Klang bewegen.\
Außerdem ist denkbar, dass mehrere Roboter eingebunden werden, um einen volleren Klang zu erzeugen und den Effekt der räumlichen Akustik besser ausnutzen zu können.
