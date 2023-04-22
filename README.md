# esphome-inchanet-wallbox
Inchanet Wallbox UART Integration for ESPHome

## Usage
[See here for how to use external components](https://esphome.io/components/external_components.html).
I've included an example also.

## inchanet_wallbox
For connection of esphome to a Inchanet Wallbox using the UART interface (contact the Inchanet for possible upgradde).

Currently, this component is designed to read all values from the wallbox and set 3-phase/1-phase charging (via a switch), max charging current (via a select) and default charging current (via a select).
