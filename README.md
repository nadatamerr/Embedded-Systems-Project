# Embedded-Systems-Project
Car Auto Parking System
This project automates the car parking system with a help of Embedded system concepts. 

## Motivation 
The problem we are trying to tackle is the struggle people face when parking their cars. Other than trying to find an empty parking spot, the driver will then try to imagine whether or not their car will fit in this specific spot. After that, they will start to position the car correctly to begin reversing into the parking space to finally park. Throughout this whole process, there are other cars that have been blocked by this person trying to park. In addition, for beginners especially, crashing the car into a parked car is a high possibility. Therefore, parking assistance mechanisms were introduced. such as parking sensors, rear cameras and finally auto parking. Auto parking is the ultimate solution for this issue, as it optimizes the whole parking process and alleviates the stress and responsibility off of the vehicle driver. Although parking sensors and cameras are very common now, only a few car manufacturers have implemented the auto-parking feature.

## Proposed Project Description
The main objective of the project is to automate vehicle parking. This can be implemented using the Nucleo STM32L432KC microcontroller and sensors. To be more specific, we will use infrared sensors to be able to detect the location of the parking space, as well as the dimensions of the space. This will be used to determine whether or not a certain parking space is of suitable size for the vehicle. If the space is suitable, the vehicle would automatically stop searching for an empty space and would start parking in that space. We will use a bluetooth connection to communicate with the vehicle from a mobile phone. This communication will only be used when the vehicle is entering the parking garage. When entering the garage, we will send a command “start.” This will alert the vehicle that they have entered the parking garage and therefore should start looking for a parking spot. This command will trigger auto parking mode.

## Design & Implementation

### Hardware requirements
| Component | Description | Operation | Configuration | Reason for Use | Photo |
| --------------- | --------------- | --------------- |--------------- | --------------- | --------------- |
| Nucleo STM32L432KC | Microcontroller | It is an ARM cortex M4 based MCU. It reads the sensor's data and provides control based on these data. | Registers configured through code with respect to the datasheet. | To control a specific function in an embedded system. | ![](https://www.st.com/bin/ecommerce/api/image.PF263436.en.feature-description-include-personalized-no-cpn-medium.jpg) |
| HC-SR04 Ultrasound Sensor | Ultrasound Sensor | It detects the distance between itself and objects by emitting ultrasound signal. | Configured through code with respect to the datasheet. | Detecting the size and location of the parking space. We used ultrasound sensor because it detects objects within a range 2cm to 400cm with a ranging accuracy that can reach up to 3mm. | ![image](https://user-images.githubusercontent.com/73887463/204675659-51f4e52d-91a2-45e1-b799-8ae1507edde2.png)|
| Dagu Motor Controller, Car and Battery | Robotic car | It is a 4-wheel-drive chassis with independent suspension for each of its spiked wheels. Each wheel is connected to a DC motor for 4-wheel-drive operation. | Dagu controller will be connected to the microcontroller, which will send commands to it. | Dagu car will simulate our project’s functionality on a smaller-scale. | ![image](https://user-images.githubusercontent.com/73887463/204682207-5bb712c9-aaf2-4c7f-a7ab-d924d5c2e790.png) ![](https://www.robotpark.com/image/cache/data/PRO/91663/91663-Dagu-4WD-Thumper-Mobile-Platform-1-700x700.jpg) |
| Keyes HM10 Bluetooth Device | Bluetooth connector | It is designed for establishing short-range wireless data communication between two microcontrollers or systems. | Receive commands through UART | Connect to phone to get command “start”| ![](https://i.ebayimg.com/images/g/J30AAOSw9r1V-hI1/s-l500.jpg) |
| Connector wires | Generic | - | - | To connect the components | ![](https://5.imimg.com/data5/HX/XA/CB/SELLER-20589996/female-to-female-wire-connector-jumper--500x500.jpg) |
| USB A-to-C Cable | Generic | - | - | To connect the microcontoller to the PC | ![](https://www.marshallheadphones.com/dw/image/v2/BCQL_PRD/on/demandware.static/-/Sites-zs-master-catalog/default/dw3bb6ff8f/images/marshall/accessories/cables/large/pos-marshall-accessories-usb-cable.png) |

### Software Requirements
* **STMCubeMX**

Through a step-by-step procedure, STM32CubeMX is a graphical tool that facilitates easy configuration of STM32 microcontrollers and microprocessors along with the generation of the relating initialization C code for the Arm® Cortex®-M core ® Device Tree for the Arm® Cortex®-A core

* **Keil uVision IDE**

Project management, a run-time environment, build tools, source code editing, and code debugging are all combined in the Vision IDE into one efficient environment. uVision is simple to use and speeds up the design of embedded applications.


## How to build the project 
You need to have CubeMx and gcc. Then, you can do the connections as illusterated in the cubMx configuration page. Finally, you can generate the code and add it to the STM32. 

For more details, check this Wiki-Page: https://github.com/nadatamerr/Embedded-Systems-Project/wiki/G7:-Car-Auto-Parking-System#design--implementation
