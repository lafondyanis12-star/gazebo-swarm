# ============================================================================
# Flight Display Module
# In-flight view, telemetry displays, and vehicle control UI
#
# Rôle dans le projet gazebo_swarm : ce CMakeLists.txt fait partie du module
# QGroundControl personnalisé (dossier qgroundcontrol-swarm-master). Il
# construit le module Qt/QML "FlyViewModule", c'est-à-dire l'écran de vol de
# QGroundControl (vue en vol, télémétrie, actions guidées, checklists, vidéo,
# et le tableau de bord d'essaim "SwarmDashboard.qml"). Ce module est compilé
# en bibliothèque statique puis lié au reste de l'application QGroundControl.
# ============================================================================

# Déclare une nouvelle cible Qt : une bibliothèque statique nommée FlyViewModule.
qt_add_library(FlyViewModule STATIC)

# Déclare le module QML associé à la cible ci-dessus : regroupe tous les
# fichiers .qml de l'interface "vue de vol" sous un même identifiant d'import
# QML (URI), avec les ressources (images, icônes) intégrées au binaire.
qt_add_qml_module(FlyViewModule
    URI QGroundControl.FlyView   # Identifiant d'import utilisé dans les fichiers QML (import QGroundControl.FlyView)
    VERSION 1.0                  # Version du module QML
    RESOURCE_PREFIX /qml         # Préfixe virtuel sous lequel les fichiers QML sont accessibles (ressources Qt)
    QML_FILES                    # Liste de tous les fichiers .qml faisant partie du module
        CustomGuidedActionsController.qml
        DefaultChecklist.qml
        FixedWingChecklist.qml
        FlightDisplayViewDummy.qml
        FlightDisplayViewVideoOutput.qml
        FlightDisplayViewQtMultimedia.qml
        FlightDisplayViewUVC.qml
        FlightDisplayViewVideo.qml
        FlyView.qml
        FlyViewAdditionalActionsButton.qml
        FlyViewAdditionalActionsList.qml
        FlyViewAdditionalActionsPanel.qml
        FlyViewAdditionalCustomActionsList.qml
        FlyViewBottomRightRowLayout.qml
        FlyViewCustomLayer.qml
        FlyViewInsetViewer.qml
        FlyViewInstrumentPanel.qml
        FlyViewGripperButton.qml
        FlyViewGripperDropPanel.qml
        FlyViewMap.qml
        FlyViewMissionCompleteDialog.qml
        FlyViewPreFlightChecklistPopup.qml
        FlyViewToolStrip.qml
        FlyViewToolStripActionList.qml

        FlyViewTopRightColumnLayout.qml
        FlyViewTopRightPanel.qml
        FlyViewVideo.qml
        FlyViewWidgetLayer.qml
        GuidedActionConfirm.qml
        GuidedActionLand.qml
        GuidedActionPause.qml
        GuidedActionRTL.qml
        GuidedActionsController.qml
        GuidedActionTakeoff.qml
        GuidedToolStripAction.qml
        GuidedValueSlider.qml
        MultiRotorChecklist.qml
        MultiVehicleList.qml
        SwarmDashboard.qml
        ObstacleDistanceOverlay.qml
        ObstacleDistanceOverlayMap.qml
        ObstacleDistanceOverlayVideo.qml
        OnScreenCameraTrackingController.qml
        OnScreenGimbalController.qml
        PreFlightBatteryCheck.qml
        PreFlightCheckList.qml
        PreFlightCheckListShowAction.qml
        PreFlightGPSCheck.qml
        PreFlightRCCheck.qml
        PreFlightSensorsHealthCheck.qml
        PreFlightSoundCheck.qml
        ProximityRadarValues.qml
        ProximityRadarVideoView.qml
        RoverChecklist.qml
        SubChecklist.qml
        TelemetryValuesBar.qml
        TerrainProgress.qml
        VehicleWarnings.qml
        VirtualJoystick.qml
        VTOLChecklist.qml
    NO_PLUGIN   # N'expose pas ce module comme plugin QML chargeable dynamiquement (lié statiquement dans l'appli)
)

# Bloc de ressources génériques : images/icônes utilisées par les vues de vol
# (ex : icône de préhenseur "Gripper", icône d'atterrissage, fond "pas de vidéo").
# Elles sont intégrées au binaire sous le préfixe virtuel "/res".
qt_add_resources(FlyViewModule flyview_resources
    PREFIX "/res"
    BASE "${CMAKE_SOURCE_DIR}/resources"
    FILES
        "${CMAKE_SOURCE_DIR}/resources/Gripper.svg"
        "${CMAKE_SOURCE_DIR}/resources/land.svg"
        "${CMAKE_SOURCE_DIR}/resources/NoVideoBackground.jpg"
        "${CMAKE_SOURCE_DIR}/resources/pause-mission.svg"
)

# Second bloc de ressources, dédié aux images utilisées directement depuis les
# fichiers QML (préfixe virtuel "/qmlimages"), séparé du bloc précédent pour
# organiser les ressources par usage.
qt_add_resources(FlyViewModule flyview_qmlimages
    PREFIX "/qmlimages"
    BASE "${CMAKE_SOURCE_DIR}/resources"
    FILES
        "${CMAKE_SOURCE_DIR}/resources/check.svg"
)
