# mpc_package
## Paczka dedykowane jest do symulatora F1TENTH opartego na symulatorze [AWSIM v1.1.0](https://github.com/PPI-PUT/autoware/tree/f1tenth)

# Model pojazdu:
Jako model pojazdu (samochodu), o który został oparty cały kontroler wybrano tzw. kinematyczny model roweru. Został on poddany reaformulacji w celu dostosowania go do realizowanego zadania - podążania za trajektorią ścieżki toru. Model przestrzenny (ang. Spatial Bicycle [3]) został sformuowany na podstawie grafki poniżej - zmiennymi stanu są: odsunięcie poprzeczne od ścieżki, błąd kąta _yaw_ oraz dystans przebytu wzdłuż trajektorii. 

![image](https://github.com/Morgaliel/mpc_package/assets/64833115/711440e3-7635-45a3-bd71-b222e1e3f613)
[Model roweru z zaznaczonym układem odniesienia do referencyjnej ścieżki. (ref: [3])

MPC zwraca jako sterowanie: kąt skrętu koła oraz prędkość wzdłuż osi pojazdu. Regulatorem prędkości jest dodatkowy regulator PID.

# Ścieżka referencyjna:
Ścieżka referencyjna została wyznaczona ręcznie jako zbiór kilkudziesięciu punktów wzdłuż trasy (gdzie punkt początkowy jest pierwyszym punktem). Rozdzielczość ścieżki ([waypoint/m]) została zwiększona/dopasowana bazując na mapie zajętości podanej jako obraz. Referencyjna prędkość oraz kąt skierowania pojazdu zostały wyznaczone z uwzględnieniem fizycznych ograniczeń: przyspieszeń wzdłużnych i poprzecznych oraz prędkości maksymalnej. Punkt od którego liczony jest horyzont predykcji wyznaczany jest jako kartezjańsko najbliższy punkt z listy punktów referencyjnych. 

# Solver:
[OSQP](https://osqp.org/docs/examples/mpc.html)

# Środowisko symulacyjne:
Całość sterowania realizowana jest w Callbacku _ground_truth_, gdzie zbierane są dane potrzebne do wyznaczenia stanu modelu (kinematycznego i potem przestrzennego). Częstotliwość wywoływania wynosi 25Hz.

Strutura projektu
```
.
mpc_package
├── f1tenth_mpc_lib                  
│   ├── MPC.py             # algorytm MPC
│   ├── spatial_bicycle_models.py   # model pojazdu
│   ├── pid.py            # regulator dynamiki wzdłużnej 
│   ├── map.py            # obsługa mapy (via matssteinweg)
│   └── reference_path.py  # generate reference path, waypoints for the assigned map (via matssteinweg)
├── f1tenth_mpc.py         # plik wykonywalny, zawierający Node sterujący ze wszystkimi callbackami, definicja problemu i wartości parametrów                
```

# Porównanie z Pure Pursuit:
Zestawienie przebiegów prędkości pojazdu w funkcji drogi przebytej wzdłuż środka toru.
![image](https://github.com/Morgaliel/mpc_package/assets/64833115/db509c6d-c070-4524-99fe-cf440dc37bcb)
(kolorem zielonym - MPC, żółtym - PP)

# Napotkane problemy:

Rozbieżność między lokalizają pozyskiwaną z _ground_truth_, a lokalizajcą pojadu w symulatorze nie pozwalała na łatwe dostosowywanie ścieżki w celach optymalizaji czasu przejazdu. W pojedunczych zakrętach trzebabyło  
Linearyzajca modelu (szczególnie np. przy większych skrętach kierownicy) niepozwala na uzyskanie płynnych i nieoscylujących rozwiązań. 
Brak znajomości parametrów pojazdu z symulatora (zastosowanego modelu, geometrii skrętu, itd.) nie pozwiliła na dokładne dobranie parametrów modelu - przy tak prostym przyjętym modelu nie sprawiało to większego problemu. 

Bibliografia: 
* [1] https://github.com/coldhenry/Model-Predictive-Control-of-Autonomous-Car
* [2] https://github.com/matssteinweg/Multi-Purpose-MPC
* [3] [Stability Conditions for Linear Time-Varying Model Predictive Control in Autonomous
Driving](http://urn.kb.se/resolve?urn=urn:nbn:se:kth:diva-220576)
* [4] [Towards Time-Optimal Race Car Driving Using Nonlinear MPC in Real-Time](https://www.researchgate.net/profile/Robin_Verschueren/publication/269860931_Towards_Time-Optimal_Race_Car_Driving_Using_Nonlinear_MPC_in_Real-Time/links/56ab66e108aeadd1bdce436b/Towards-Time-Optimal-Race-Car-Driving-Using-Nonlinear-MPC-in-Real-Time.pdf?origin=publication_detail)




Autorzy:
* Jan Iwaszkiewicz - Rudoszański 
* Maksymilian Jaruga
