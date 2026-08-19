import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QGroundControl
import QGroundControl.Controls

// Tableau de bord de l'essaim de drones : affiche la liste de tous les véhicules connectés
// (MultiVehicleManager de QGroundControl) avec leur position GPS, altitude, vitesse, cap,
// batterie et statut de connexion/armement. Permet aussi de sélectionner un ou plusieurs
// drones et de leur envoyer des commandes groupées (armer/désarmer, décoller, atterrir,
// changer d'altitude, régler la vitesse max) ainsi que de basculer le flux vidéo affiché
// entre les caméras des différents drones. Gère également une élection automatique du
// "leader" de l'essaim (le drone le plus apte à prendre le rôle de référence).
Rectangle {
    id:     swarmDashboard
    color:  "#CC000000"
    radius: 8
    width:  300
    height: Math.min(swarmColumn.implicitHeight + 40, 600)

    // Liste des véhicules (drones) actuellement connus par QGroundControl
    property var  _vehicles:      QGroundControl.multiVehicleManager.vehicles
    // Identifiants des drones actuellement sélectionnés dans l'UI (pour les commandes groupées)
    property var  selectedIds:    []
    // Identifiant du drone dont la caméra est actuellement affichée (-1 = aucune)
    property int  activeCamDrone: -1

    // Change le flux vidéo affiché pour utiliser la caméra du drone "droneId"
    // (redirige la source vidéo UDP de QGroundControl vers le port correspondant à ce drone)
    function switchCamera(droneId) {
        var ports = { 1: "0.0.0.0:5600", 2: "0.0.0.0:5601", 3: "0.0.0.0:5602" }
        var port = ports[droneId]
        if (!port) return
        QGroundControl.settingsManager.videoSettings.videoSource.rawValue =
            QGroundControl.settingsManager.videoSettings.udp264VideoSource
        QGroundControl.settingsManager.videoSettings.udpUrl.rawValue = port
        activeCamDrone = droneId
        console.log("Camera switched to Drone " + droneId + " on " + port)
    }
    // Identifiant du drone actuellement élu "leader" de l'essaim (-1 = aucun)
    property int  leaderId:       -1
    // Explication textuelle de la dernière décision d'élection du leader (pour affichage debug/info)
    property string leaderReason: ""
    // Altitude cible (en mètres, AGL = au-dessus du sol) utilisée pour le décollage/changement d'altitude
    property real takeoffAlt:     10
    // Vitesse maximale (m/s) appliquée aux drones sélectionnés
    property real maxSpeed:       5

    // Temporise l'envoi de la commande de vitesse pour éviter de spammer le lien MAVLink
    // pendant que l'utilisateur déplace le curseur du slider
    Timer {
        id: speedTimer
        interval: 600; repeat: false
        onTriggered: swarmDashboard.applySpeed()
    }

    // Indique si le drone "id" fait partie de la sélection courante
    function isSelected(id) { return selectedIds.indexOf(id) >= 0 }

    // Ajoute ou retire un drone de la sélection au clic sur sa carte
    function toggleSelect(id) {
        var arr = selectedIds.slice()
        var idx = arr.indexOf(id)
        if (idx >= 0) arr.splice(idx, 1)
        else arr.push(id)
        selectedIds = arr
    }

    // Sélectionne tous les drones actuellement connus
    function selectAll() {
        var arr = []
        for (var i = 0; i < _vehicles.count; i++) arr.push(_vehicles.get(i).id)
        selectedIds = arr
    }

    // Désélectionne tous les drones
    function selectNone()   { selectedIds = [] }
    // Sélectionne uniquement le drone leader (s'il existe)
    function selectLeader() { selectedIds = leaderId >= 0 ? [leaderId] : [] }

    // Envoie une commande de vol ("arm"/"disarm"/"takeoff"/"land") à tous les drones
    // actuellement sélectionnés, via les fonctions de mode guidé de QGroundControl
    function dispatch(action) {
        for (var i = 0; i < _vehicles.count; i++) {
            var v = _vehicles.get(i)
            if (!isSelected(v.id)) continue
            if      (action === "arm")     v.armed = true
            else if (action === "disarm")  v.armed = false
            else if (action === "takeoff") v.guidedModeTakeoff(takeoffAlt)
            else if (action === "land")    v.guidedModeLand()
        }
    }

    // Applique la vitesse maximale (maxSpeed) aux drones sélectionnés et armés en envoyant
    // une commande MAVLink DO_CHANGE_SPEED (id 179) directement au véhicule
    function applySpeed() {
        for (var i = 0; i < _vehicles.count; i++) {
            var v = _vehicles.get(i)
            if (!isSelected(v.id) || !v.armed) continue
            v.sendMavCommand(1, 179, true, 1, maxSpeed, -1, 0, 0, 0, 0)
        }
    }

    // Change l'altitude des drones sélectionnés (armés et déjà en vol) pour atteindre
    // l'altitude cible "takeoffAlt", en calculant le delta par rapport à l'altitude relative
    // actuelle et en utilisant le mode guidé de changement d'altitude de QGroundControl.
    // MAVLink DO_REPOSITION avec altitude AMSL absolue
    // ASL = altitude du point de départ (home) + altitude cible AGL
    function applyAltitude() {
        for (var i = 0; i < _vehicles.count; i++) {
            var v = _vehicles.get(i)
            if (!isSelected(v.id) || !v.armed) continue
            if (v.altitudeRelative.value < 0.5) continue
            var delta = takeoffAlt - v.altitudeRelative.value
            v.guidedModeChangeAltitude(delta, false)
        }
    }
    // Variante non utilisée (conservée pour référence) : envoie une commande MAVLink
    // DO_REPOSITION brute avec l'altitude absolue AMSL calculée manuellement
    function applyAltitude_UNUSED() {
        for (var i = 0; i < _vehicles.count; i++) {
            var v = _vehicles.get(i)
            if (!isSelected(v.id) || !v.armed) continue
            if (v.altitudeRelative.value < 0.5) continue
            var homeAlt  = v.altitudeAMSL.value - v.altitudeRelative.value
            var targetAMSL = homeAlt + takeoffAlt
            v.sendMavCommand(
                1,
                192,
                true,
                -1,
                1,
                0,
                0,
                v.coordinate.latitude,
                v.coordinate.longitude,
                targetAMSL
            )
        }
    }

    // Formate une valeur numérique avec "dec" décimales ; retourne "--" si la valeur est invalide
    function fmt(val, dec) {
        try { return Number(val).toFixed(dec) }
        catch(e) { return "--" }
    }

    // Recherche un véhicule par son identifiant dans la liste des drones connectés
    function getVehicleById(id) {
        for (var i = 0; i < _vehicles.count; i++)
            if (_vehicles.get(i).id === id) return _vehicles.get(i)
        return null
    }

    // Calcule un score de "qualité de vol" utilisé pour l'élection du leader :
    // -1 = perdu/déconnecté, 0 = désarmé, 1 = armé (au sol/hold), 2 = armé et en vol actif
    function flightScore(v) {
        if (!v || v.connectionLost) return -1
        if (v.armed && v.flightMode !== "Hold") return 2
        if (v.armed) return 1
        return 0
    }

    // Élit automatiquement le drone "leader" de l'essaim : celui avec le meilleur score de
    // vol parmi les drones connectés (à égalité, l'id le plus petit est choisi). Gère aussi
    // le "failover" : si le leader actuel disparaît/perd la connexion, un nouveau leader est
    // choisi automatiquement et la raison est mémorisée dans leaderReason.
    function electLeader() {
        if (!_vehicles || _vehicles.count === 0) { leaderId = -1; return }
        var bestId = -1
        for (var i = 0; i < _vehicles.count; i++) {
            var v = _vehicles.get(i)
            if (v.connectionLost) continue
            if (bestId === -1) { bestId = v.id; continue }
            var vScore    = flightScore(v)
            var bestScore = flightScore(getVehicleById(bestId))
            if (vScore > bestScore || (vScore === bestScore && v.id < bestId))
                bestId = v.id
        }
        if (bestId !== leaderId) {
            if      (leaderId === -1) leaderReason = "Initial election -> Drone " + bestId
            else if (bestId   === -1) leaderReason = "No active drone"
            else                      leaderReason = "Failover: Drone " + leaderId + " lost -> Drone " + bestId
            leaderId = bestId
        }
    }

    // Relance périodiquement l'élection du leader (toutes les 2s) pour détecter les pertes
    // de connexion ou changements de statut de vol sans attendre un événement explicite
    Timer { interval: 2000; running: true; repeat: true; onTriggered: swarmDashboard.electLeader() }

    // Réagit à l'ajout/suppression d'un drone dans la liste des véhicules connus
    // (ex: connexion/déconnexion) en relançant l'élection du leader et en resélectionnant tout
    Connections {
        target: _vehicles
        function onCountChanged() { swarmDashboard.electLeader(); swarmDashboard.selectAll() }
    }

    // Initialisation au chargement du composant : élit un leader et sélectionne tous les drones
    Component.onCompleted: { electLeader(); selectAll() }

    // Zone défilante contenant tout le contenu du tableau de bord (liste des drones,
    // élection du leader, paramètres de vol, boutons de commande)
    Flickable {
        id:           flick
        anchors.fill: parent
        anchors.margins: 4
        contentHeight: swarmColumn.implicitHeight + 20
        clip:          true
        ScrollBar.vertical: ScrollBar { policy: ScrollBar.AsNeeded }

        // Colonne principale empilant toutes les sections du tableau de bord
        Column {
            id:      swarmColumn
            width:   flick.width - 8
            anchors.horizontalCenter: parent.horizontalCenter
            spacing: 6
            topPadding: 6

            // Titre du panneau
            QGCLabel {
                text: "SWARM DASHBOARD"
                font.bold: true; color: "white"
                font.pointSize: ScreenTools.defaultFontPointSize * 1.2
            }

            // Nombre de drones actuellement connectés
            QGCLabel { text: "Drones: " + _vehicles.count; color: "#00FF00" }

            // Génère une carte d'information pour chaque drone présent dans _vehicles
            Repeater {
                model: _vehicles

                // Carte représentant un drone : cliquable pour le sélectionner/désélectionner,
                // mise en évidence si c'est le leader ou s'il est sélectionné
                Rectangle {
                    width:  swarmColumn.width
                    height: droneCol.implicitHeight + 12
                    radius: 4
                    color:  isSelected(object.id) ? "#1A3A5A" : "#1A1A2E"
                    border.color: object.id === leaderId ? "#f59e0b" :
                                  isSelected(object.id)  ? "#3b82f6" : "#444"
                    border.width: (object.id === leaderId || isSelected(object.id)) ? 2 : 1

                    // Référence au véhicule courant du Repeater (raccourci de lecture)
                    property var _drone: object

                    // Zone cliquable sur toute la carte : bascule la sélection du drone
                    MouseArea {
                        anchors.fill: parent
                        onClicked: swarmDashboard.toggleSelect(object.id)
                        cursorShape: Qt.PointingHandCursor
                    }

                    Column {
                        id:              droneCol
                        anchors.left:    parent.left
                        anchors.right:   parent.right
                        anchors.top:     parent.top
                        anchors.margins: 6
                        spacing:         6

                        // Ligne d'en-tête de la carte : case de sélection, pastille de statut,
                        // identifiant, mode de vol, statut armé/désarmé, bouton caméra, badge leader
                        RowLayout {
                            width: parent.width
                            spacing: 8

                            // Case à cocher visuelle indiquant si le drone est sélectionné
                            Rectangle {
                                width: 14; height: 14; radius: 3
                                color:        isSelected(_drone.id) ? "#3b82f6" : "transparent"
                                border.color: isSelected(_drone.id) ? "#3b82f6" : "#888"
                                border.width: 1.5
                                QGCLabel {
                                    anchors.centerIn: parent
                                    text: "v"; color: "white"
                                    font.pointSize: ScreenTools.defaultFontPointSize * 0.7
                                    visible: isSelected(_drone.id)
                                }
                            }

                            // Pastille de couleur indiquant l'état de connexion/armement
                            // (gris = connexion perdue, vert = armé, rouge = désarmé)
                            Rectangle {
                                width: 10; height: 10; radius: 5
                                color: _drone && _drone.connectionLost ? "#666" :
                                       _drone && _drone.armed ? "#00FF00" : "#FF4444"
                            }

                            // Identifiant du drone
                            QGCLabel {
                                text: _drone ? "Drone " + _drone.id : ""
                                color: "white"; Layout.fillWidth: true
                            }

                            // Mode de vol courant (ex: Hold, Guided...)
                            QGCLabel {
                                text: _drone ? _drone.flightMode : ""
                                color: "#AAAAAA"
                                font.pointSize: ScreenTools.defaultFontPointSize * 0.9
                            }

                            // Statut armé/désarmé du drone
                            QGCLabel {
                                text:  _drone && _drone.armed ? "ARMED" : "DISARMED"
                                color: _drone && _drone.armed ? "#00FF00" : "#FF4444"
                                font.pointSize: ScreenTools.defaultFontPointSize * 0.85
                            }

                            // Bouton "CAM" : bascule le flux vidéo affiché sur la caméra de ce drone
                            Rectangle {
                                width: 32; height: 16; radius: 8
                                color: activeCamDrone === _drone.id ? "#1a5c1a" : "#1a1a3a"
                                border.color: activeCamDrone === _drone.id ? "#00FF00" : "#444"
                                border.width: 1
                                QGCLabel {
                                    anchors.centerIn: parent
                                    text: "CAM"
                                    color: activeCamDrone === _drone.id ? "#00FF00" : "#888"
                                    font.pointSize: ScreenTools.defaultFontPointSize * 0.7
                                    font.bold: true
                                }
                                MouseArea {
                                    anchors.fill: parent
                                    onClicked: swarmDashboard.switchCamera(_drone.id)
                                    cursorShape: Qt.PointingHandCursor
                                }
                            }

                            // Badge "LEADER" affiché uniquement sur la carte du drone élu leader
                            Rectangle {
                                visible: _drone && _drone.id === leaderId
                                width: 46; height: 16; radius: 8; color: "#78350f"
                                QGCLabel {
                                    anchors.centerIn: parent
                                    text: "LEADER"; color: "#fbbf24"
                                    font.pointSize: ScreenTools.defaultFontPointSize * 0.7
                                    font.bold: true
                                }
                            }
                        }

                        // Grille de télémétrie (3 colonnes) : latitude, longitude, altitude,
                        // vitesse sol, cap et niveau de batterie du drone
                        Grid {
                            columns: 3; width: parent.width; spacing: 4

                            // Construit la liste des valeurs de télémétrie à afficher pour ce drone
                            Repeater {
                                model: [
                                    { label: "LAT",    val: fmt(_drone.coordinate.latitude,    6) },
                                    { label: "LON",    val: fmt(_drone.coordinate.longitude,   6) },
                                    { label: "ALT(m)", val: fmt(_drone.altitudeRelative.value, 1) },
                                    { label: "SPD",    val: fmt(_drone.groundSpeed.value,      1) },
                                    { label: "HDG",    val: fmt(_drone.heading.value,          0) },
                                    // Pourcentage de batterie restant (protégé par try/catch
                                    // au cas où aucune batterie n'est reportée par le drone)
                                    { label: "BAT(%)", val: (function() {
                                        try { return _drone.batteries.count > 0 ?
                                            fmt(_drone.batteries.get(0).percentRemaining.value, 0) : "--"
                                        } catch(e) { return "--" } })() }
                                ]

                                // Petite tuile affichant un libellé et sa valeur
                                Rectangle {
                                    width: (droneCol.width - 8) / 3
                                    height: 34; radius: 4; color: "#0D0D1A"
                                    Column {
                                        anchors.centerIn: parent; spacing: 2
                                        QGCLabel {
                                            text: modelData.label; color: "#666"
                                            font.pointSize: ScreenTools.defaultFontPointSize * 0.7
                                            anchors.horizontalCenter: parent.horizontalCenter
                                        }
                                        QGCLabel {
                                            text: modelData.val
                                            color: {
                                                // Colore la valeur de batterie selon son niveau
                                                // (vert > 50%, orange > 20%, rouge sinon)
                                                if (modelData.label !== "BAT(%)") return "#DDDDDD"
                                                var b = parseInt(modelData.val)
                                                if (isNaN(b)) return "#DDDDDD"
                                                return b > 50 ? "#00FF00" : b > 20 ? "#FFA500" : "#FF4444"
                                            }
                                            font.pointSize: ScreenTools.defaultFontPointSize * 0.85
                                            font.bold: true
                                            anchors.horizontalCenter: parent.horizontalCenter
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // Séparateur visuel
            Rectangle { width: swarmColumn.width; height: 1; color: "#444" }

            // Section "élection du leader" : affiche le leader courant, les critères
            // d'élection et la raison de la dernière décision (élection initiale ou failover)
            Rectangle {
                width: swarmColumn.width
                height: leaderCol.implicitHeight + 10
                color: "#0D1A0D"; radius: 4
                border.color: "#f59e0b"; border.width: 1
                visible: _vehicles.count > 0

                Column {
                    id: leaderCol
                    anchors { left: parent.left; right: parent.right; top: parent.top; margins: 6 }
                    spacing: 3

                    QGCLabel {
                        text: "LEADER ELECTION"; color: "#f59e0b"; font.bold: true
                        font.pointSize: ScreenTools.defaultFontPointSize * 0.85
                    }
                    // Affiche l'identifiant du leader courant (lié à la propriété leaderId)
                    QGCLabel {
                        text: leaderId >= 0 ? "Leader: Drone " + leaderId : "No active leader"
                        color: "#fbbf24"
                        font.pointSize: ScreenTools.defaultFontPointSize * 0.85
                    }
                    // Rappel des règles de priorité utilisées par electLeader()
                    QGCLabel {
                        text: "Criteria: in-flight > armed > disarmed — auto failover"
                        color: "#555"; font.pointSize: ScreenTools.defaultFontPointSize * 0.75
                    }
                    // Détail de la dernière décision d'élection (visible seulement si renseigné)
                    QGCLabel {
                        visible: leaderReason !== ""
                        text: leaderReason; color: "#888"
                        font.pointSize: ScreenTools.defaultFontPointSize * 0.75
                        wrapMode: Text.WordWrap; width: parent.width
                    }
                }
            }

            // Séparateur visuel
            Rectangle { width: swarmColumn.width; height: 1; color: "#444" }

            // Section "paramètres de vol" : réglages d'altitude cible et de vitesse max,
            // appliqués aux drones sélectionnés via les boutons/sliders ci-dessous
            Rectangle {
                width: swarmColumn.width
                height: paramsCol.implicitHeight + 10
                color: "#0D0D1A"; radius: 4
                border.color: "#334"; border.width: 1

                Column {
                    id: paramsCol
                    anchors { left: parent.left; right: parent.right; top: parent.top; margins: 6 }
                    spacing: 6

                    QGCLabel {
                        text: "FLIGHT PARAMETERS"; color: "#AAAAAA"; font.bold: true
                        font.pointSize: ScreenTools.defaultFontPointSize * 0.85
                    }

                    // Ligne affichant l'altitude cible actuelle
                    RowLayout {
                        width: parent.width
                        QGCLabel {
                            text: "Target altitude (AGL)"; color: "#888"
                            font.pointSize: ScreenTools.defaultFontPointSize * 0.85
                            Layout.fillWidth: true
                        }
                        QGCLabel {
                            text: takeoffAlt.toFixed(0) + " m"; color: "#00FF00"; font.bold: true
                            font.pointSize: ScreenTools.defaultFontPointSize * 0.85
                        }
                    }

                    // Curseur pour régler l'altitude cible (takeoffAlt), de 1 à 50 m
                    Slider {
                        width: parent.width; from: 1; to: 50; stepSize: 1; value: takeoffAlt
                        onValueChanged: takeoffAlt = value
                    }

                    // Boutons pour décoller à l'altitude cible ou changer l'altitude en vol
                    RowLayout {
                        width: parent.width
                        QGCButton {
                            text: "TAKEOFF " + takeoffAlt.toFixed(0) + "m"
                            Layout.fillWidth: true
                            enabled: selectedIds.length > 0
                            opacity: selectedIds.length > 0 ? 1.0 : 0.4
                            onClicked: swarmDashboard.dispatch("takeoff")
                        }
                        QGCButton {
                            text: "CHANGE ALT"
                            Layout.fillWidth: true
                            enabled: selectedIds.length > 0
                            opacity: selectedIds.length > 0 ? 1.0 : 0.4
                            onClicked: swarmDashboard.applyAltitude()
                        }
                    }

                    // Ligne affichant la vitesse maximale actuelle
                    RowLayout {
                        width: parent.width
                        QGCLabel {
                            text: "Max speed"; color: "#888"
                            font.pointSize: ScreenTools.defaultFontPointSize * 0.85
                            Layout.fillWidth: true
                        }
                        QGCLabel {
                            text: maxSpeed.toFixed(0) + " m/s"; color: "#00FF00"; font.bold: true
                            font.pointSize: ScreenTools.defaultFontPointSize * 0.85
                        }
                    }

                    // Curseur pour régler la vitesse max (maxSpeed), de 1 à 20 m/s ;
                    // relance speedTimer pour appliquer la commande MAVLink avec un léger délai
                    Slider {
                        width: parent.width; from: 1; to: 20; stepSize: 1; value: maxSpeed
                        onValueChanged: { maxSpeed = value; speedTimer.restart() }
                    }
                }
            }

            // Séparateur visuel
            Rectangle { width: swarmColumn.width; height: 1; color: "#444" }

            // Boutons de sélection rapide : tous les drones, aucun, ou seulement le leader
            RowLayout {
                width: swarmColumn.width; spacing: 4
                QGCButton { text: "ALL";    Layout.fillWidth: true; onClicked: swarmDashboard.selectAll() }
                QGCButton { text: "NONE";   Layout.fillWidth: true; onClicked: swarmDashboard.selectNone() }
                QGCButton { text: "LEADER"; Layout.fillWidth: true; onClicked: swarmDashboard.selectLeader() }
            }

            // Résumé textuel de la sélection courante avant les boutons de commande
            QGCLabel {
                text: {
                    if (selectedIds.length === 0) return "COMMANDS — no drone selected"
                    if (selectedIds.length === _vehicles.count) return "COMMANDS — all drones"
                    return "COMMANDS — Drone(s) " + selectedIds.join(", ")
                }
                color: "#AAAAAA"; font.pointSize: ScreenTools.defaultFontPointSize * 0.9
            }

            // Boutons ARMER / DÉSARMER pour les drones sélectionnés (désactivés si aucune sélection)
            RowLayout {
                width: swarmColumn.width; spacing: 4
                opacity: selectedIds.length > 0 ? 1.0 : 0.4

                QGCButton {
                    text: "ARM"; Layout.fillWidth: true
                    enabled: selectedIds.length > 0
                    onClicked: swarmDashboard.dispatch("arm")
                }
                QGCButton {
                    text: "DISARM"; Layout.fillWidth: true
                    enabled: selectedIds.length > 0
                    onClicked: swarmDashboard.dispatch("disarm")
                }
            }

            // Bouton ATTERRIR pour les drones sélectionnés
            QGCButton {
                text: "LAND"; width: swarmColumn.width
                opacity: selectedIds.length > 0 ? 1.0 : 0.4
                enabled: selectedIds.length > 0
                onClicked: swarmDashboard.dispatch("land")
            }

            // Espace vide en bas de la colonne pour respirer visuellement
            Item { height: 6 }
        }
    }
}
