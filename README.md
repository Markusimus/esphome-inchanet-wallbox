# esphome-inchanet-wallbox
Inchanet Wallbox UART Integration for ESPHome

## Usage
[See here for how to use external components](https://esphome.io/components/external_components.html).
I've included an example also.

## Description
There is used a UART interface for connection to the Inchanet Wallbox (contact the Inchanet for possible upgrade).

Currently, this component is designed to read all values from the wallbox and set 3-phase/1-phase charging (via a switch), max charging current (via a select) and default charging current (via a select).


## [Unreleased]

### Added
- Support for hardware flow control via a configurable GPIO pin to improve communication reliability. (#...)
- Initial vendor information request to ensure the wallbox is correctly identified before operation. (#...)

### Changed
- Refactored the UART communication protocol handling to be more robust and maintainable. (#...)
- Improved distinction between warnings and serious errors reported by the wallbox. (#...)

## Changelog
