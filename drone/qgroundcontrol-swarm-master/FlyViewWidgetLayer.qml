import QtQuick
import QtQuick.Controls
import QtQuick.Dialogs
import QtQuick.Layouts

import QtLocation
import QtPositioning
import QtQuick.Window
import QtQml.Models

import QGroundControl
import QGroundControl.Controls
import QGroundControl.FlyView
import QGroundControl.FlightMap
import QGroundControl.Viewer3D

// Ce composant est la couche de superposition (overlay) qui regroupe tous les widgets/outils
// affichés par-dessus la carte dans la vue "Fly View" de QGroundControl : bandeau d'outils,
// panneaux d'informations, joystick virtuel, échelle de carte, boîtes de dialogue de vol,
// et le tableau de bord de l'essaim (SwarmDashboard) ajouté pour ce projet.
Item {
    id: _root

    // Marges/insets fournis par le parent (zone déjà occupée par d'autres éléments d'UI)
    property var    parentToolInsets
    // Marges/insets calculées par ce composant, exposées au parent pour éviter les chevauchements
    property var    totalToolInsets:        _totalToolInsets
    // Référence vers le contrôle de carte (FlightMap) affiché en arrière-plan
    property var    mapControl

    // Véhicule (drone) actuellement sélectionné/actif dans QGroundControl
    property var    _activeVehicle:         QGroundControl.multiVehicleManager.activeVehicle
    // Contrôleurs de plan de vol (mission, geofence, points de ralliement) pour la vue Fly View
    property var    _planMasterController:  globals.planMasterControllerFlyView
    property var    _missionController:     _planMasterController.missionController
    property var    _geoFenceController:    _planMasterController.geoFenceController
    property var    _rallyPointController:  _planMasterController.rallyPointController
    // Contrôleur du mode "guidé" (déplacements manuels commandés depuis l'UI)
    property var    _guidedController:      globals.guidedControllerFlyView
    // Marges/espacements calculés à partir de la taille de police (pour s'adapter au DPI)
    property real   _margins:               ScreenTools.defaultFontPixelWidth / 2
    property real   _toolsMargin:           ScreenTools.defaultFontPixelWidth * 0.75
    // Zone centrale de la vue (utilisée comme viewport de référence)
    property rect   _centerViewport:        Qt.rect(0, 0, width, height)
    property real   _rightPanelWidth:       ScreenTools.defaultFontPixelWidth * 30
    property real   _layoutMargin:          ScreenTools.defaultFontPixelWidth * 0.75
    property bool   _layoutSpacing:         ScreenTools.defaultFontPixelWidth
    property bool   _showSingleVehicleUI:   true

    // Regroupe toutes les marges (insets) occupées par les différents widgets afin que
    // la carte et les autres éléments d'UI puissent s'ajuster sans se superposer
    QGCToolInsets {
        id:                     _totalToolInsets
        leftEdgeTopInset:       toolStrip.leftEdgeTopInset
        leftEdgeCenterInset:    toolStrip.leftEdgeCenterInset
        leftEdgeBottomInset:    virtualJoystickMultiTouch.visible ? virtualJoystickMultiTouch.leftEdgeBottomInset : parentToolInsets.leftEdgeBottomInset
        rightEdgeTopInset:      topRightPanel.rightEdgeTopInset
        rightEdgeCenterInset:   topRightPanel.rightEdgeCenterInset
        rightEdgeBottomInset:   bottomRightRowLayout.rightEdgeBottomInset
        topEdgeLeftInset:       toolStrip.topEdgeLeftInset
        topEdgeCenterInset:     mapScale.topEdgeCenterInset
        topEdgeRightInset:      topRightPanel.topEdgeRightInset
        bottomEdgeLeftInset:    virtualJoystickMultiTouch.visible ? virtualJoystickMultiTouch.bottomEdgeLeftInset : parentToolInsets.bottomEdgeLeftInset
        bottomEdgeCenterInset:  bottomRightRowLayout.bottomEdgeCenterInset
        bottomEdgeRightInset:   virtualJoystickMultiTouch.visible ? virtualJoystickMultiTouch.bottomEdgeRightInset : bottomRightRowLayout.bottomEdgeRightInset
    }

    // Panneau de widgets affiché en haut à droite (ex: infos véhicule, télémétrie condensée)
    FlyViewTopRightPanel {
        id:                     topRightPanel
        anchors.top:            parent.top
        anchors.right:          parent.right
        maximumHeight:          parent.height - (bottomRightRowLayout.height + _margins * 4)

        property real topEdgeRightInset:    height + _layoutMargin
        property real rightEdgeTopInset:    width + _layoutMargin
        property real rightEdgeCenterInset: rightEdgeTopInset
    }

    // Variante en colonne du panneau haut-droite, visible seulement si topRightPanel est caché
    FlyViewTopRightColumnLayout {
        id:                 topRightColumnLayout
        anchors.top:        parent.top
        anchors.right:      parent.right
        spacing:            _layoutSpacing
        visible:           !topRightPanel.visible

        property real topEdgeRightInset:    childrenRect.height + _layoutMargin
        property real rightEdgeTopInset:    width + _layoutMargin
        property real rightEdgeCenterInset: rightEdgeTopInset
    }

    // Rangée de widgets/boutons affichée en bas à droite de l'écran
    FlyViewBottomRightRowLayout {
        id:                 bottomRightRowLayout
        anchors.bottom:     parent.bottom
        anchors.right:      parent.right
        spacing:            _layoutSpacing

        property real bottomEdgeRightInset:     height + _layoutMargin
        property real bottomEdgeCenterInset:    bottomEdgeRightInset
        property real rightEdgeBottomInset:     width + _layoutMargin
    }

    // Boîte de dialogue affichée automatiquement quand une mission est terminée
    FlyViewMissionCompleteDialog {
        missionController:      _missionController
        geoFenceController:     _geoFenceController
        rallyPointController:   _rallyPointController
    }

    //-- Joystick virtuel (contrôle tactile du drone à l'écran)
    // Chargé dynamiquement (Loader) et affiché uniquement si l'option est activée dans les
    // réglages et que le lien de communication n'est pas en haute latence
    Loader {
        id:                         virtualJoystickMultiTouch
        z:                          QGroundControl.zOrderTopMost + 1
        anchors.right:              parent.right
        anchors.rightMargin:        anchors.leftMargin
        height:                     Math.min(parent.height * 0.25, ScreenTools.defaultFontPixelWidth * 16)
        visible:                    _virtualJoystickEnabled && !QGroundControl.videoManager.fullScreen && !(_activeVehicle ? _activeVehicle.usingHighLatencyLink : false)
        anchors.bottom:             parent.bottom
        anchors.bottomMargin:       bottomLoaderMargin
        anchors.left:               parent.left
        anchors.leftMargin:         ( y > toolStrip.y + toolStrip.height ? toolStrip.width / 2 : toolStrip.width * 1.05 + toolStrip.x)
        source:                     "qrc:/qml/QGroundControl/FlyView/VirtualJoystick.qml"
        active:                     _virtualJoystickEnabled && !(_activeVehicle ? _activeVehicle.usingHighLatencyLink : false)

        property real bottomEdgeLeftInset:     parent.height-y
        // Réglages utilisateur du joystick virtuel (auto-centrage, mode gaucher)
        property bool autoCenterThrottle:      QGroundControl.settingsManager.appSettings.virtualJoystickAutoCenterThrottle.rawValue
        property bool leftHandedMode:          QGroundControl.settingsManager.appSettings.virtualJoystickLeftHandedMode.rawValue
        // Indique si le joystick virtuel est activé dans les réglages de l'application
        property bool _virtualJoystickEnabled: QGroundControl.settingsManager.appSettings.virtualJoystick.rawValue
        property real bottomEdgeRightInset:    parent.height-y
        // Marge basse en fonction de la vue caméra incrustée (PIP) si elle est visible
        property var  _pipViewMargin:          _pipView.visible ? parentToolInsets.bottomEdgeLeftInset + ScreenTools.defaultFontPixelHeight * 2 :
                                               bottomRightRowLayout.height + ScreenTools.defaultFontPixelHeight * 1.5

        property var  bottomLoaderMargin:      _pipViewMargin >= parent.height / 2 ? parent.height / 2 : _pipViewMargin

        // La largeur est difficile à obtenir directement, d'où cette astuce qui peut ne pas
        // fonctionner dans tous les cas
        property real leftEdgeBottomInset:  visible ? bottomEdgeLeftInset + width/18 - ScreenTools.defaultFontPixelHeight*2 : 0
        property real rightEdgeBottomInset: visible ? bottomEdgeRightInset + width/18 - ScreenTools.defaultFontPixelHeight*2 : 0
        property real rootWidth:            _root.width
        property var  itemX:                virtualJoystickMultiTouch.x   // position X réelle à l'écran

        // Propage la largeur et la position réelles au composant chargé pour qu'il puisse
        // calculer correctement les zones tactiles
        onRootWidthChanged: virtualJoystickMultiTouch.status == Loader.Ready && visible ? virtualJoystickMultiTouch.item.uiTotalWidth = rootWidth : undefined
        onItemXChanged:     virtualJoystickMultiTouch.status == Loader.Ready && visible ? virtualJoystickMultiTouch.item.uiRealX = itemX : undefined

        // Logique déclenchée une fois le composant chargé
        onLoaded: {
            if (virtualJoystickMultiTouch.visible) {
                // Active le mode calibration et transmet les dimensions actuelles
                virtualJoystickMultiTouch.item.calibration = true
                virtualJoystickMultiTouch.item.uiTotalWidth = rootWidth
                virtualJoystickMultiTouch.item.uiRealX = itemX
            } else {
                virtualJoystickMultiTouch.item.calibration = false
            }
        }
    }

    // Bandeau d'outils vertical à gauche (accès rapide aux actions de vol : décollage,
    // atterrissage, checklist pré-vol, etc.)
    FlyViewToolStrip {
        id:                     toolStrip
        anchors.left:           parent.left
        anchors.top:            parent.top
        z:                      QGroundControl.zOrderWidgets
        maxHeight:              parent.height - y - parentToolInsets.bottomEdgeLeftInset - _toolsMargin
        visible:                !QGroundControl.videoManager.fullScreen

        // Ouvre la popup de checklist pré-vol quand l'utilisateur clique sur le bouton dédié
        onDisplayPreFlightChecklist: {
            if (!preFlightChecklistLoader.active) {
                preFlightChecklistLoader.active = true
            }
            preFlightChecklistLoader.item.open()
        }

        property real topEdgeLeftInset:     visible ? y + height : 0
        property real leftEdgeTopInset:     visible ? x + width : 0
        property real leftEdgeCenterInset:  leftEdgeTopInset
    }

    // Tableau de bord de l'essaim de drones (composant personnalisé du projet) : affiche la
    // liste des drones connectés avec leur position, batterie, statut et permet d'envoyer des
    // commandes groupées (armer, décoller, atterrir...). Voir SwarmDashboard.qml.
    // Positionné juste à droite du bandeau d'outils, visible seulement s'il y a au moins un
    // véhicule connecté.
    SwarmDashboard {
    	id:                 swarmDashboard
    	anchors.left:       toolStrip.right
    	anchors.leftMargin: _toolsMargin
    	anchors.top:        toolStrip.bottom
    	anchors.topMargin:  _toolsMargin
    	z:                  QGroundControl.zOrderWidgets
    	visible:            QGroundControl.multiVehicleManager.vehicles.count > 0
    }

    // Affiche les avertissements liés au véhicule (alarmes, erreurs critiques) au centre de l'écran
    VehicleWarnings {
        anchors.centerIn:   parent
        z:                  QGroundControl.zOrderTopMost
    }

    // Échelle graphique de la carte (distance représentée par une longueur à l'écran)
    MapScale {
        id:                 mapScale
        anchors.left:       toolStrip.right
        anchors.leftMargin: _toolsMargin
        anchors.top:        parent.top
        mapControl:         _mapControl
        autoHide:           true
        visible:            !ScreenTools.isTinyScreen && QGroundControl.corePlugin.options.flyView.showMapScale && QGCViewer3DManager.displayMode !== QGCViewer3DManager.View3D && mapControl.pipState.state === mapControl.pipState.fullState

        property real topEdgeCenterInset: visible ? y + height : 0
    }

    // Chargeur différé (lazy loading) de la popup de checklist pré-vol : n'est instancié
    // qu'au moment où l'utilisateur en a besoin (active devient true)
    Loader {
        id: preFlightChecklistLoader
        sourceComponent: preFlightChecklistPopup
        active: false
    }

    // Définit le composant de la popup de checklist pré-vol, utilisé par le Loader ci-dessus
    Component {
        id: preFlightChecklistPopup
        FlyViewPreFlightChecklistPopup {
        }
    }
}
