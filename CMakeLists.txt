catkin_install_python(PROGRAMS
  scripts/voice_listener.py
  scripts/speech_recognition.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)