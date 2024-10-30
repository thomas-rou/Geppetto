
## Name
This is the official readme for the Geppetto project

## Description
The purpose of this project is to develop a proof of concept for planetary exploration using a multi-robot system, equipped with the minimum sensors required by the Canadian Space Agency (CSA). This project, focused on research and education in space exploration, aims to demonstrate the effectiveness and viability of a simple autonomous multi-robot system in a controlled indoor environment simulating planetary exploration conditions. The robots can autonomously explore an unknown area, roughly the size of a room, using only the sensors specified by the CSA: the IMU ("Inertial Measurement Unit"), the 3D camera, the RGB camera, and the LiDAR ("Light Detection and Ranging"). The operator is able to monitor the data in real-time via a web interface, with limited control over starting and stopping the operations. The system will generate an accurate map of the explored area.

## Installation
To start using the project execute command: 
"docker compose up" to start the control station and the gazebo simulation.
Then, to start the limo robots, connect to two using the SSH protocol and run the following command:
"~/.launch.sh"

## Usage
From the website, the user is able to start an exploration mission, stop said mission, ask robots to identify themselves and monitor the mapping of the surroundings in real time while also having the option to review previous mission logs and generated maps.

For additional information on usage read the DocumentationDuProjet-107-offre-PDR document

## Coding guidelines
For Javascript and Typescript, the [Airbnb Standards](https://github.com/airbnb/javascript) have been used along with the [default prettier options](https://prettier.io/docs/en/options.html).

For Python, the [Google coding guidlines](https://google.github.io/styleguide/pyguide.html) have been chosen.

The branch and commits naming best practices have been taken from
[this blog](https://medium.com/@shinjithkanhangad/git-good-best-practices-for-branch-naming-and-commit-messages-a903b9f08d68).

## Support
Contact "loic8558" and/or "velistic" on Discord for help or questions regarding the usage or installation of this project

## Authors and acknowledgment
Project Authors: Ely Cheikh Abyss, Omar Benzekri, Abdul-Wahab Chaarani, Loïc Nguemegne, Thomas Rouleau and Ivan Samoylenko 

Project made possible with help of Antoine Robillard, Mark Bong, Guillaume Ricard and Benjamin De Leener

S/o to our sponsors: Pizzeria Geppetto Beaubien.
Don't hesitate to visit them: 2510 Rue Beaubien E, Montréal, QC H1Y 1G2 
Mention promo code: "Zavelafedouzegomen" for a 10% discount.


