Cinder-KCB2
===========

Cinder wrapper for Kinect 2 Common Bridge

===========

To enable face tracking on a TinderBox-generated project, add
this line to "Post-Build Events"

xcopy /e /i /y "$(KINECTSDK20_DIR)Redist\Face\$(PlatformTarget)\NuiDatabase" "$(OutDir)NuiDatabase"
