# STEMRobotCar
__Robot Car for area mapping__

# Implementation details
* Path Finding Algorithm

```
  Mark current block as seen
  if obstacles
     mark forward as obstacles
     
  find possible direction(s)
  for each direction
     if the next block is unseen
         choose that direction
     if no direction is chose
         choose one direction randomly
```
