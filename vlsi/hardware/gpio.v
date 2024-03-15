// 2021-2024 Philipp Ruppel

`include "gpio_core.v"

module gpio #(
    parameter CHANNELS = 4
) (
    `make_mem_ports(),
    inout wire [CHANNELS-1:0] pins
);

    wire [CHANNELS-1:0] gpio_i;
    wire [CHANNELS-1:0] gpio_o;
    wire [CHANNELS-1:0] gpio_t;

    gpio_core #(
        .CHANNELS(CHANNELS)
    ) gpio_core_inst (
        `forward_mem_signals(),
        .pins_i(gpio_i),
        .pins_o(gpio_o),
        .pins_t(gpio_t)
    );

    TRELLIS_IO #( .DIR("BIDIR") ) io_inst [CHANNELS-1:0] ( .B(pins), .I(gpio_o), .O(gpio_i), .T(gpio_t) );

endmodule
