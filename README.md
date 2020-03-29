# midi-syncbox
MIDI synchronization box for timed control of multiple hardware musical instruments.

When performing live electronic music it could be useful to have hardware instruments synchronized by the same clock signal.
Commercial devices with an elevate clock precision and advanced features exist but they usually have an high cost due to use of high-end electronic components.
A device which can give basic features can be easily built using the Arduino built in components (ATMEL ATMEGA328P-PU microcontroller and Oscillator).

## Basic Functionalities

- Management of external clock signal (MIDI, DIN, USB) and its propagation towards multiple MIDI out ports
- Generation of an internal clock signal, tempo value is tweakable by means of a potentiometer
- Selection of the clock source and synchronization control with play/pause and stop buttons

## Advanced Functionalities

- BPM display shows the selected tempo when clock is generated internally
- Tap tempo button is used instead of the potentiometer to select tempo
- Shuffle patterns generation for the synchronization signal

## Getting Started

### Prerequisites

### Installing

## Running the tests

## Deployment

## Authors

* **Andrea Campisi** - [PurpleBooth](https://github.com/AndreaCamper)
* **Jacopo Foglietti** - [PurpleBooth](https://github.com/jfoglietti)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Arduino Timer Interrupts mechanism [PurpleBooth](https://https://www.instructables.com/id/Arduino-Timer-Interrupts/)

