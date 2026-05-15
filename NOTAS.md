El RSSI con lrwpan (801.15.4 en NS3) es complicado de sacar.

En NS3 siempre se busca poder utlizar el sistema de trazas para obtener medidas (es la manera limpia).

El problema es que no existe traza que me de el RSSI (PowerRX) como tal, solo es posible obtener el SINR (PowerRX/Interference+Noise)

Existe otra opción, la cual es usar el callback de la pila de protocolos simulada en NS3. Es decir, sacar las medidas como si "vivieramos" sobre la L2/L1 OSI, y utlizar el servicio para obtenerla. Nos podemos basar en lr-wpan-data.cc

Otra opción es usar la traza gain del ns3::SpectrumChannel, la cual te da los suficientes parámetros para calcularlo, salvo por la potencia de transmisión, la cual está por defecto a 0 en lr-wpan-phy.cc (m_phyPIBAttributes.phyTransmitPower = 0;). 

La opción más fina es usar el callback sobre la pila de protocolos, tal y como se hace de forma extendida en redes IOT. Esto es así debido a la limitada MTU del protocolo (127B), poner la pila TCP/IP sobre una MTU así es un completo desperdicio.

En resumen, vamos a tener que trabajar con primitivas de servicio. Según la documentación NS3, las primitivas disponibles en el simulador son:

```Plano de Datos
MCPS-DATA (Request, Confirm, Indication)
```

```Plano de Control
MLME-START (Request, Confirm)
MLME-SCAN (Request, Confirm)
MLME-BEACON-NOFIFY (Indication)
MLME-ASSOCIATE.Request (Request, Confirm, Response, Indication)
MLME-POLL (Confirm)
MLME-COMM-STATUS (Indication)
MLME-SYNC (Request)
MLME-SYNC-LOSS (Indication)
MLME-SET (Request, Confirm)
MLME-GET (Request, Confirm)
```

```Primitivas Capa Física
PLME-CCA (Request, Confirm)
PD-DATA (Request, Confirm, Indication)
PLME-SET-TRX-STATE (Request, Confirm)
PLME-SET (Request, Confirm)
PLME-GET (Request, Confirm)
```

