
## Inspiration
In today's society, many people have jobs that require them to sit for extended periods, making it crucial to drink enough water throughout the day. Prolonged sitting can lead to a loss of body fluids, necessitating water replenishment to maintain the body's balance. Additionally, drinking water can help reduce fatigue, increase energy, and boost productivity. As a result, it is vital for individuals who spend long hours sitting to maintain adequate water intake.

## What it does
Magic Bottle is a smart water bottle designed to monitor and record users' daily water consumption. It utilizes a nau7802 pressure sensor to measure the remaining water volume after each sip and an LIS2DH12TR three-axis inertial sensor to determine the bottle's current status. The data is then displayed on an SSD1306 screen, providing users with real-time updates on their hydration levels.

## How we built it
We built the Magic Bottle using a SAMW25 microcontroller and the FreeRTOS operating system. The hardware components include three peripherals: an SSD1306 display, a nau7802 pressure sensor, and an LIS2DH12TR three-axis inertial sensor. The PCB design was created using Altium Design software.

## Challenges we ran into
The most significant challenge we faced was with the I2C communication protocol. After powering up the completed board, we discovered that the I2C communication was not returning valid information. Upon further investigation, we found that one of the components was preventing the SCL line from being pulled up, rendering communication impossible.

## Accomplishments that we're proud of
We are proud of our team's ability to successfully design and build a functioning PCB board without encountering any issues upon powering it up. This achievement demonstrates our team's technical expertise and ability to overcome challenges during the development process.

## What we learned
Throughout the Magic Bottle project, we gained valuable experience in designing and building a smart water bottle from scratch. We also learned how to troubleshoot hardware communication issues, ultimately leading to a better understanding of the I2C protocol and its potential pitfalls.

## What's next for Magic Bottle
In the next phase of Magic Bottle development, we will focus on optimizing its structure to enhance the integration and overall unity with the bottle itself. We will also work on improving the algorithm to ensure more sensitive detection and lower error rates in monitoring water consumption.

Furthermore, we will redesign the system to increase its stability for extended periods. Currently, the system can operate smoothly for about an hour, but running it for more than 12 hours may result in issues with MQTT sending and receiving. By addressing these limitations, we aim to create a more reliable and user-friendly hydration tracking device that users can depend on throughout the day.



Link of node-red

https://lovely-caspian-gull-5069.flowforge.cloud/?#flow/9dcdfca5b65e56ed

Youtube Link

https://youtu.be/S6ohsKgnpBM

Altium Link

https://upenn-eselabs.365.altium.com/designs/B8637907-61C4-4523-9F5F-885A0351ED5C
