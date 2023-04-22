# esphome-inchanet-wallbox
Inchanet Wallbox UART Integration for ESPHome

## Usage
[See here for how to use external components](https://esphome.io/components/external_components.html).
I've included an example also.

## Description
There is used a UART interface for connection to the Inchanet Wallbox (contact the Inchanet for possible upgrade).

Currently, this component is designed to read all values from the wallbox and set 3-phase/1-phase charging (via a switch), max charging current (via a select) and default charging current (via a select).
