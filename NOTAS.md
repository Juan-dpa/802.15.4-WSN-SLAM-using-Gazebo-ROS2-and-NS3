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

Modo depuración, para lanzar:

```bash
NS_GLOBAL_VALUE="RngRun=1" NS_LOG="SlamDataCollector=level_all|prefix_all" ./ns3 run "topology --pcap=true"
```

Se ha añadido la posibilidad de usar un modelo de perdidas de propagación mucho más fino. El modelo Hybrid permite utilizar diferentes algoritmos empíricos de manera automática, en base a las condiciones de cada transmisión. Referenciando al manual de NS3:

```
This model includes Hata model, COST231, ITU-R P.1411 (short range communications), ITU-R P.1238 (indoor communications), which are combined in order to be able to evaluate the pathloss under different scenarios, in detail:

Environments: urban, suburban, open-areas;
frequency: from 200 uo to 2600 MHz
short range communications vs long range communications
Node position respect to buildings: indoor, outdoor and hybrid (indoor <-> outdoor)
Building penetretation loss
floors, etc...
```

Además, se ha refactorizado el código, para separar la lógica de lectura de los CSV en un CsvHelper, el cual facilita el código. Además, se han añadido 2 scripts para la obtención de los dados estrcturales del mapa usado en Gazebo (turtlebot3_house). El scrapper obtiene el csv, a partir del model.sdf y los .dae que requiere dicho sdf. Se puede validar gráficamente que el resultado represente correctamente al modelo visto en gazebo, usando validator.py.

```bash
NS_GLOBAL_VALUE="RngRun=1" NS_LOG="SlamDataCollector=level_all|prefix_all:HybridBuildingsPropagationLossModel=level_all|prefix_all:CSV_HELPER=level_all|prefix_all" ./ns3 run "topology --pcap=false --outputFile=scratch/Proyecto_ROS2_WSN/Outputs/slam_dataset_run1.csv"
```

C++
// 1. Instanciamos una variable aleatoria para el Jitter (ej. +/- 15 milisegundos)
Ptr<UniformRandomVariable> jitter = CreateObject<UniformRandomVariable>();
jitter->SetAttribute("Min", DoubleValue(-0.015));
jitter->SetAttribute("Max", DoubleValue(0.015));

for (uint32_t i = 0; i < numTransmitters; i++) {
    // Simulamos que cada placa arranca en un momento aleatorio entre 0 y 200ms
    Ptr<UniformRandomVariable> bootDelay = CreateObject<UniformRandomVariable>();
    double bootTime = bootDelay->GetValue(0.0, 0.2); 

    uint32_t seqNum = 0;
    
    // Fíjate que empezamos en bootTime, no en 0.0
    for (double t = bootTime; t <= lastTrajectoryTime; t += txInterval) {
        
        // 2. Le sumamos el clock drift / jitter a cada disparo individual
        double instanteAleatorio = t + jitter->GetValue();
        Time txTime = Seconds(instanteAleatorio);

        // ... (Creación del paquete y buffer igual que antes) ...

        Simulator::Schedule(txTime, &LrWpanMac::McpsDataRequest, macLayer, params, p);
        seqNum++;
    }
}