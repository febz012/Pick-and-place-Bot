# eYRC_Bot
## A pick-and-place bot powered using an FPGA.

The Bot is built for deployment in an arena that abstracts an agricultural field.This robot traverses the arena, senses the environment and picks and places necessary supplies from the supply unit to the field.The arena is set up using a printed track.

The bot senses the track using a line follower sensor and traverses the whole field.Specific locations in the arena are given a distinct colour code(RED,GREEN,BLUE).The supply unit consists of a number of cubical boxes.Each of these boxes are also given a colour code matching the locations in arena.The bot has to supply the boxes with a colour code to its matching location in the arena.The colour code is sensed using a colour sensor.

The pick-and-place mechanism is implemented using an electromagnet.Each box is pre installed with a ferromagnetic material.
