#!/bin/bash

for i in {1..30}; do
    RNG=$i
    # --no-build evita que intenten recompilar o blo>
    NS_GLOBAL_VALUE="RngRun=$RNG" ./ns3 run "topolog>
done

wait