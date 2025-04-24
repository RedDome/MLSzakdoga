#!/bin/bash

echo "Ellenőrizzük, hogy a szenzor adatok elérhetők-e..."

if rostopic list | grep -q "/scan"; then
  echo "LiDAR adat elérhető a /scan témán."
else
  echo "Hiba: LiDAR adat nem elérhető!"
fi

if rostopic list | grep -q "/ultrasonic"; then
  echo "Ultrahangos szenzor adat elérhető a /ultrasonic témán."
else
  echo "Hiba: Ultrahangos szenzor adat nem elérhető!"
fi

if rostopic list | grep -q "/infrared"; then
  echo "Infravörös szenzor adat elérhető a /infrared témán."
else
  echo "Hiba: Infravörös szenzor adat nem elérhető!"
fi
