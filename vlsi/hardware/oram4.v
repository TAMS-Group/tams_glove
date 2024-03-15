// 2021-2024 Philipp Ruppel

`include "utils.v"
`include "packet_fifo.v"

module oram #(
  parameter CAMERA_BIT_DEPTH = 12,
) (
  `make_mem_ports(),

  inout wire [7:0] pin_ram_d,
  inout wire pin_ram_q,
  output wire pin_ram_ce,
  output reg pin_ram_clk,
  output wire pin_ram_reset,

  input wire ram_clk,
  input wire ram_clk_shift,

  input wire pin_camera_pixclk,
  input wire [CAMERA_BIT_DEPTH-1:0] pin_camera_data,
  input wire pin_camera_frame_valid,
  input wire pin_camera_line_valid,

  input wire packet_out_clk,
  input wire packet_out_pop,
  output wire packet_out_nempty,
  output wire [63:0] packet_out_data,
  output wire packet_out_end
);

  reg [CAMERA_BIT_DEPTH-1:0] black_level_reg = 0;
  reg [CAMERA_BIT_DEPTH-1:0] black_level_cam = 0;

  reg [CAMERA_BIT_DEPTH-1:0] cap_camera_data;
  reg cap_camera_frame_valid;
  reg cap_camera_line_valid;

  always @(negedge pin_camera_pixclk) begin
    cap_camera_data <= pin_camera_data;
    cap_camera_frame_valid <= pin_camera_frame_valid;
    cap_camera_line_valid <= pin_camera_line_valid;
  end

  reg [CAMERA_BIT_DEPTH-1:0] reg_camera_data;
  reg reg_camera_frame_valid;
  reg reg_camera_line_valid;

  always @(posedge pin_camera_pixclk) begin
    reg_camera_data <= cap_camera_data;
    reg_camera_frame_valid <= cap_camera_frame_valid;
    reg_camera_line_valid <= cap_camera_line_valid;
  end

  localparam PIXEL_FIFO_WIDTH = CAMERA_BIT_DEPTH + 2;

  wire [PIXEL_FIFO_WIDTH-1:0] cam_fifo_in_data_packed;
  assign cam_fifo_in_data_packed[CAMERA_BIT_DEPTH-1:0] = reg_camera_data;
  assign cam_fifo_in_data_packed[CAMERA_BIT_DEPTH+0] = reg_camera_frame_valid;
  assign cam_fifo_in_data_packed[CAMERA_BIT_DEPTH+1] = reg_camera_line_valid;
  wire cam_fifo_in_full;
  wire cam_fifo_in_shift = !cam_fifo_in_full;
  wire cam_fifo_out_nempty;
  wire cam_fifo_out_pop = cam_fifo_out_nempty;
  wire [PIXEL_FIFO_WIDTH-1:0] cam_fifo_out_data_packet;
  wire [CAMERA_BIT_DEPTH-1:0] cam_fifo_out_pixel = cam_fifo_out_data_packet[CAMERA_BIT_DEPTH-1:0];
  wire cam_fifo_out_frame_valid = cam_fifo_out_data_packet[CAMERA_BIT_DEPTH+0];
  wire cam_fifo_out_line_valid = cam_fifo_out_data_packet[CAMERA_BIT_DEPTH+1];
  async_fifo #(
    .WIDTH(PIXEL_FIFO_WIDTH),
    .DEPTH(64)
  ) cam_fifo_inst (
    .in_clk(pin_camera_pixclk),
    .in_shift(cam_fifo_in_shift),
    .in_data(cam_fifo_in_data_packed),
    .in_full(cam_fifo_in_full),
    .out_clk(ram_clk),
    .out_pop(cam_fifo_out_pop),
    .out_data(cam_fifo_out_data_packet),
    .out_nempty(cam_fifo_out_nempty)
  );

  localparam CAM_BUFFER_SIZE = 1024 * 8;

  reg [7:0] cam_buffer_data [CAM_BUFFER_SIZE-1:0];

  reg [7:0] cam_buffer_data_out = 0;
  reg [$clog2(CAM_BUFFER_SIZE)-1:0] cam_buffer_data_raddr = 0;

  reg [7:0] cam_buffer_data_in = 0;
  reg [$clog2(CAM_BUFFER_SIZE)-1:0] cam_buffer_data_waddr = 0;
  reg cam_buffer_data_wstrobe = 0;

  reg [$clog2(CAM_BUFFER_SIZE)-1:0] cam_buffer_write_pointer = 0;
  reg [$clog2(CAM_BUFFER_SIZE)-1:0] cam_buffer_read_pointer = 0;
  reg [$clog2(CAM_BUFFER_SIZE)-1:0] cam_buffer_temp_pointer = 0;

  localparam CAM_LUT_SIZE = (1 << CAMERA_BIT_DEPTH);
  reg [7:0] cam_lut_data [CAM_LUT_SIZE-1:0];
  integer i;
  
  initial begin
    for (i = 0; i < CAM_LUT_SIZE; i = i + 1) begin
      
      cam_lut_data[i] = ($rtoi($sqrt(i * (1.0 / (CAM_LUT_SIZE - 1))) * 255 + 0.5) ^ 8'b10101010);
    end
  end

  reg cam_zero_valid = 0;
  reg cam_zero_line = 0;
  reg cam_zero_frame = 0;
  reg [CAMERA_BIT_DEPTH-1:0] cam_zero_pixel = 0;

  always @(posedge ram_clk) begin
    cam_zero_valid <= 0;
    if (cam_fifo_out_pop) begin
      cam_zero_valid <= 1;
      cam_zero_line <= cam_fifo_out_line_valid;
      cam_zero_frame <= cam_fifo_out_frame_valid;
      if (cam_fifo_out_pixel < black_level_cam)
        cam_zero_pixel <= 0;
      else
        cam_zero_pixel <= cam_fifo_out_pixel - black_level_cam;
    end
  end

  reg cam_proc_valid = 0;
  reg cam_proc_line = 0;
  reg cam_proc_frame = 0;
  reg [7:0] cam_proc_pixel = 0;

  always @(posedge ram_clk) begin
    cam_proc_valid <= 0;
    if (cam_zero_valid) begin
      cam_proc_valid <= 1;
      cam_proc_line <= cam_zero_line;
      cam_proc_frame <= cam_zero_frame;
      cam_proc_pixel <= cam_lut_data[cam_zero_pixel];
    end
  end

  reg [7:0] cam_proc_frame_valid_hist = 0;
  reg cam_proc_frame_valid_start = 0;

  

  reg packet_fifo_in_push = 0;
  reg [63:0] packet_fifo_in_data = 0;
  reg packet_fifo_in_end = 0;
  wire packet_fifo_in_full;
  packet_fifo #(
      .WORD_SIZE(64),
      .FIFO_DEPTH(1 * 1024)
  ) packet_fifo_inst (
      .in_clk(ram_clk),
      .in_full(packet_fifo_in_full),
      .in_shift(packet_fifo_in_push && !packet_fifo_in_full),
      .in_data(packet_fifo_in_data),
      .in_end(packet_fifo_in_end),
      .out_clk(packet_out_clk),
      .out_pop(packet_out_pop),
      .out_nempty(packet_out_nempty),
      .out_data(packet_out_data),
      .out_end(packet_out_end)
  );

  

  localparam PACKET_WORDS = 8;
  localparam PACKET_BYTES = PACKET_WORDS * 8;

  reg [23:0] packet_count = 0;
  reg [23:0] packet_index = 0;

  reg [7:0] packet_word = 0;

  reg [23:0] packet_payload_address = 0;

  

  reg data_n_oe;
  wire [7:0] data_in;
  reg [7:0] data_out;
  (* IO_TYPE="LVCMOS18", TERMINATION="50", HYSTERESIS="OFF", SLEWRATE="FAST", DRIVE=8 *)
  TRELLIS_IO #( .DIR("BIDIR") ) io_d [7:0] ( .B(pin_ram_d), .I(data_out), .O(data_in), .T(data_n_oe) );
  
  reg mask_n_oe;
  reg mask_out;
  wire mask_in;
  (* IO_TYPE="LVCMOS18", TERMINATION="50", HYSTERESIS="OFF", SLEWRATE="FAST", DRIVE=8 *)
  TRELLIS_IO #( .DIR("BIDIR") ) io_nask ( .B(pin_ram_q), .I(mask_out), .O(mask_in), .T(mask_n_oe) );
  
  reg chip_not_select_out;
  
  (* IO_TYPE="LVCMOS18", SLEWRATE="FAST", DRIVE=8 *)
  TRELLIS_IO #( .DIR("OUTPUT") ) io_ce ( .B(pin_ram_ce), .I(chip_not_select_out) );
  
  reg phase = 0;
  reg ram_clk_o = 0;
  always @(posedge ram_clk_shift) begin
    
    ram_clk_o <= phase;
  end
  (* IO_TYPE="LVCMOS18", SLEWRATE="SLOW", DRIVE=8 *)
  TRELLIS_IO #( .DIR("OUTPUT") ) io_ram_clk ( .B(pin_ram_clk), .I(ram_clk_o) );
  
  reg reg_reset = 0;
  
  (* IO_TYPE="LVCMOS18", SLEWRATE="FAST", DRIVE=8 *)
  TRELLIS_IO #( .DIR("OUTPUT") ) io_ram_reset ( .B(pin_ram_reset), .I(reg_reset) );
  
  localparam RAM_STATE_IDLE = 0;

  localparam RAM_STATE_REGISTER_READ = 1;
  localparam RAM_STATE_REGISTER_WRITE = 2;
  localparam RAM_STATE_DATA_READ = 3;
  localparam RAM_STATE_DATA_WRITE = 4;
  localparam RAM_STATE_ACKNOWLEDGE = 5;
  localparam RAM_STATE_RESET = 6;
  localparam RAM_STATE_READ_PACKET = 7;

  localparam RAM_STATE_CAM_WRITE = 8;

  localparam CAMERA_BURST_LENGTH = 128;

  reg [23:0] cam_ram_pointer = 0;

  reg [7:0] ram_state = RAM_STATE_IDLE;
  
  reg [15:0] step = 0;

  reg x_req = 0;
  reg x_ack = 0;

  reg [31:0] x_addr = 0;
  reg [31:0] x_wdata = 0;
  reg [3:0] x_wstrb = 0;

  reg [31:0] x_rdata = 0;

  reg x_req_ram = 0;

  reg enable_packet_tx = 0;
  reg enable_packet_tx_ram = 0;

  reg [11:0] packet_checksum = 0;
  wire [11:0] packet_checksum_next = (packet_checksum * 41 + data_in);

  always @(posedge ram_clk) begin

    

    enable_packet_tx_ram <= enable_packet_tx;

    x_req_ram <= x_req;

    phase <= ~phase;

    step <= step + 1;
  
    data_n_oe <= 1;
    data_out <= 0;
    mask_n_oe <= 1;
    mask_out <= 0;

    chip_not_select_out <= 1;

    x_ack <= 0;

    packet_fifo_in_push <= 0;

    case (ram_state)

      RAM_STATE_IDLE: begin

        if (phase == 1) begin

          if (x_req_ram) begin

            case (x_addr & 32'h0f000000)

              32'h02000000: begin
                if (x_wstrb != 0)
                  ram_state <= RAM_STATE_REGISTER_WRITE;
                else
                  ram_state <= RAM_STATE_REGISTER_READ;
                step <= 0;
              end

              32'h03000000: begin
                if (x_wstrb != 0)
                  ram_state <= RAM_STATE_DATA_WRITE;
                else
                  ram_state <= RAM_STATE_DATA_READ;
                step <= 0;
              end

            endcase

          end else if (
            (cam_buffer_read_pointer / CAMERA_BURST_LENGTH) != (cam_buffer_write_pointer / CAMERA_BURST_LENGTH)
            &&
            (cam_buffer_read_pointer / CAMERA_BURST_LENGTH) + 1 != (cam_buffer_write_pointer / CAMERA_BURST_LENGTH)
          ) begin

            ram_state <= RAM_STATE_CAM_WRITE;
            step <= 0;

          end else if (
            enable_packet_tx_ram 
            && !packet_fifo_in_full 
            && (cam_ram_pointer > packet_index * PACKET_BYTES) 
            && packet_index < packet_count
          ) begin

            if (packet_index == 0) begin

              if (packet_word == 0) begin
                packet_fifo_in_data <= info_header;
                packet_fifo_in_push <= 1;
                packet_fifo_in_end <= 0;
                packet_word <= 1;
              end else begin
                packet_fifo_in_data <= info_contents;
                packet_fifo_in_push <= 1;
                packet_fifo_in_end <= 1;
                packet_index <= 1;
                packet_word <= 0;
              end

            end else begin

              if (packet_word >= PACKET_WORDS) begin

                packet_word <= 0;
                packet_index <= packet_index + 1;

                packet_fifo_in_data[19:0] <= packet_index;
                packet_fifo_in_data[31:20] <= packet_checksum;

                packet_fifo_in_end <= 1;
                packet_fifo_in_push <= 1;

              end else begin

                packet_word <= packet_word + 1;

                if (packet_word == 0) begin
                  packet_fifo_in_data[63:32] <= data_header;
                  packet_payload_address <= (packet_index - 1) * PACKET_BYTES;
                  packet_checksum <= packet_index;
                end 

                ram_state <= RAM_STATE_READ_PACKET;
                step <= 0;

              end

            end

          end

        end

      end

      RAM_STATE_CAM_WRITE: begin

        chip_not_select_out <= 0;

        case (step)
    
          
          
          
          0: begin
            data_out <= 8'hA0;
            data_n_oe <= 0;
          end
          1: begin
            data_out <= 8'hA0;
            data_n_oe <= 0;
          end
          
          
          2: begin
            data_out <= 8'h00;
            data_n_oe <= 0;
          end
          3: begin
            data_out <= cam_ram_pointer[23:16];
            data_n_oe <= 0;
          end
          4: begin
            data_out <= cam_ram_pointer[15:8];
            data_n_oe <= 0;
          end
          5: begin
            data_out <= cam_ram_pointer[7:0];
            data_n_oe <= 0;
          end
          6: begin
            cam_buffer_temp_pointer <= cam_buffer_read_pointer;
            cam_buffer_data_raddr <= cam_buffer_read_pointer;
            cam_buffer_read_pointer <= cam_buffer_read_pointer + CAMERA_BURST_LENGTH;
          end

        endcase

        if (step >= 13) begin
          cam_buffer_data_raddr <= cam_buffer_data_raddr + 1;
        end

        if (step >= 14 && step < 14 + CAMERA_BURST_LENGTH) begin
          data_out <= cam_buffer_data_out;
          data_n_oe <= 0;
          mask_out <= 0;
          mask_n_oe <= 0;
          cam_ram_pointer <= cam_ram_pointer + 1;
        end

        if (step == 14 + CAMERA_BURST_LENGTH) begin
          chip_not_select_out <= 1;
          ram_state <= RAM_STATE_IDLE;
        end

      end

      RAM_STATE_READ_PACKET: begin

        chip_not_select_out <= 0;

        case (step)

          
          0: begin
            data_out <= 8'h20;
            data_n_oe <= 0;
          end
          1: begin
            data_out <= 8'h20;
            data_n_oe <= 0;
          end
          
          
          2: begin
            data_out <= 8'h00;
            data_n_oe <= 0;
            
          end
          3: begin
            data_out <= packet_payload_address[23:16];
            
            data_n_oe <= 0;
          end
          4: begin
            data_out <= packet_payload_address[15:8];
            
            data_n_oe <= 0;
          end
          5: begin
            data_out <= packet_payload_address[7:0];
            
            data_n_oe <= 0;
          end
          6: begin
            packet_payload_address <= packet_payload_address + 8;
          end
          
          
          10: begin
            packet_fifo_in_data[7:0] <= data_in;
            if (!mask_in) 
              step <= step;
            else
              packet_checksum <= packet_checksum_next;
          end
          11: begin
            packet_fifo_in_data[15:8] <= data_in;
            packet_checksum <= packet_checksum_next;
          end
          12: begin
            packet_fifo_in_data[23:16] <= data_in;
            if (!mask_in) 
              step <= step;
            else
              packet_checksum <= packet_checksum_next;
          end
          13: begin
            packet_fifo_in_data[31:24] <= data_in;
            packet_checksum <= packet_checksum_next;
            packet_fifo_in_end <= 0;
            packet_fifo_in_push <= 1;
          end
          14: begin
            packet_fifo_in_data[39:32] <= data_in;
            if (!mask_in) 
              step <= step;
            else
              packet_checksum <= packet_checksum_next;
          end
          15: begin
            packet_fifo_in_data[47:40] <= data_in;
            packet_checksum <= packet_checksum_next;
          end
          16: begin
            packet_fifo_in_data[55:48] <= data_in;
            if (!mask_in) 
              step <= step;
            else
              packet_checksum <= packet_checksum_next;
          end
          17: begin
            packet_fifo_in_data[63:56] <= data_in;
            packet_checksum <= packet_checksum_next;
            chip_not_select_out <= 1;
          end
          18: begin
            ram_state <= RAM_STATE_IDLE;
            chip_not_select_out <= 1;
          end

        endcase

      end

      RAM_STATE_DATA_READ: begin

        chip_not_select_out <= 0;

        case (step)

          
          0: begin
            data_out <= 8'h00;
            data_n_oe <= 0;
          end
          1: begin
            data_out <= 8'h00;
            data_n_oe <= 0;
          end
          
          
          2: begin
            data_out <= 8'h00;
            data_n_oe <= 0;
          end
          3: begin
            data_out <= x_addr[23:16];
            data_n_oe <= 0;
          end
          4: begin
            data_out <= x_addr[15:8];
            data_n_oe <= 0;
          end
          5: begin
            data_out <= x_addr[7:0];
            data_n_oe <= 0;
          end
          
          
          10: begin
            x_rdata[7:0] <= data_in;
            if (!mask_in)
              step <= 10;
          end
          11: begin
            x_rdata[15:8] <= data_in;
          end
          12: begin
            x_rdata[23:16] <= data_in;
            
          end
          13: begin
            x_rdata[31:24] <= data_in;
            ram_state <= RAM_STATE_ACKNOWLEDGE;
            chip_not_select_out <= 1;
          end

        endcase

      end

      RAM_STATE_DATA_WRITE: begin

        chip_not_select_out <= 0;

        case (step)
    
          
          
          
          0: begin
            data_out <= 8'h80;
            data_n_oe <= 0;
          end
          1: begin
            data_out <= 8'h80;
            data_n_oe <= 0;
          end
          
          
          2: begin
            data_out <= 8'h00;
            data_n_oe <= 0;
          end
          3: begin
            data_out <= x_addr[23:16];
            data_n_oe <= 0;
          end
          4: begin
            data_out <= x_addr[15:8];
            data_n_oe <= 0;
          end
          5: begin
            data_out <= x_addr[7:0];
            data_n_oe <= 0;
          end
          
          
          14: begin
            data_out <= x_wdata[7:0];
            data_n_oe <= 0;
            mask_out <= ~x_wstrb[0];
            mask_n_oe <= 0;
          end
          15: begin
            data_out <= x_wdata[15:8];
            data_n_oe <= 0;
            mask_out <= ~x_wstrb[1];
            mask_n_oe <= 0;
          end
          16: begin
            data_out <= x_wdata[23:16];
            data_n_oe <= 0;
            mask_out <= ~x_wstrb[2];
            mask_n_oe <= 0;
          end
          17: begin
            data_out <= x_wdata[31:24];
            data_n_oe <= 0;
            mask_out <= ~x_wstrb[3];
            mask_n_oe <= 0;
            ram_state <= RAM_STATE_ACKNOWLEDGE;
            
          end

        endcase

      end

      RAM_STATE_REGISTER_READ: begin

        chip_not_select_out <= 0;

        case (step) 

          
          0: begin
            data_out <= 8'h40;
            data_n_oe <= 0;
          end
          1: begin
            data_out <= 8'h40;
            data_n_oe <= 0;
          end
          
          
          2: begin
            data_out <= 8'h00;
            data_n_oe <= 0;
          end
          3: begin
            data_out <= 8'h00;
            data_n_oe <= 0;
          end
          4: begin
            data_out <= 8'h00;
            data_n_oe <= 0;
          end
          5: begin
            data_out <= (x_addr >> 2);
            data_n_oe <= 0;
          end
          
          
          15: begin
            x_rdata[7:0] <= data_in;
          end
          16: begin
            x_rdata[15:8] <= data_in;
          end
          17: begin
            x_rdata[23:16] <= data_in;
            
          end
          18: begin
            x_rdata[31:24] <= data_in;
            ram_state <= RAM_STATE_ACKNOWLEDGE;
            chip_not_select_out <= 1;
          end

        endcase

      end

      RAM_STATE_REGISTER_WRITE: begin

        chip_not_select_out <= 0;

        case (step)
      
          
          
          
          0: begin
            data_out <= 8'hC0;
            data_n_oe <= 0;
          end
          1: begin
            data_out <= 8'hC0;
            data_n_oe <= 0;
          end
          
          
          2: begin
            data_out <= 8'h00;
            data_n_oe <= 0;
          end
          3: begin
            data_out <= 8'h00;
            data_n_oe <= 0;
          end
          4: begin
            data_out <= 8'h00;
            data_n_oe <= 0;
          end
          5: begin
            data_out <= (x_addr >> 2);
            data_n_oe <= 0;
          end
          
          
          6: begin
            data_out <= x_wdata[7:0];
            data_n_oe <= 0;
          end
          7: begin
            data_out <= 0;
            data_n_oe <= 0;
            
            
          end
          8: begin
            ram_state <= RAM_STATE_ACKNOWLEDGE;
            chip_not_select_out <= 1;
          end

        endcase

      end

      RAM_STATE_ACKNOWLEDGE: begin
        x_ack <= 1;
        if (!x_req_ram) begin
          ram_state <= RAM_STATE_RESET;
        end
      end

      RAM_STATE_RESET: begin
        ram_state <= RAM_STATE_IDLE;
      end

      default: begin
        ram_state <= RAM_STATE_IDLE;
      end
          
    endcase

    

    if (cam_buffer_data_wstrobe) begin
      cam_buffer_data[cam_buffer_data_waddr] <= cam_buffer_data_in;
    end
    cam_buffer_data_wstrobe <= 0;

    if (cam_proc_valid) begin
      cam_proc_frame_valid_hist <= {cam_proc_frame_valid_hist, cam_proc_frame};
      if (!cam_proc_line && (cam_proc_frame_valid_hist == 0)) begin
        cam_proc_frame_valid_start <= 1;
        if (!cam_proc_frame_valid_start) begin
          cam_buffer_write_pointer <= ((cam_buffer_write_pointer + (CAMERA_BURST_LENGTH * 3 - 1)) / CAMERA_BURST_LENGTH * CAMERA_BURST_LENGTH);
        end
      end
      if (cam_proc_line) begin

        

        cam_buffer_write_pointer <= cam_buffer_write_pointer + 1;

        cam_buffer_data_wstrobe <= 1;
        cam_buffer_data_in <= cam_proc_pixel + cam_buffer_data_in * 31;

        cam_buffer_data_waddr <= cam_buffer_write_pointer + 1;

        if (cam_proc_frame_valid_start) begin
          cam_buffer_data_in <= cam_proc_pixel;
          cam_ram_pointer <= 0;
          cam_buffer_read_pointer <= 0;
          cam_buffer_write_pointer <= 0;
          packet_index <= 0;
          packet_word <= 0;
          cam_proc_frame_valid_start <= 0;
          black_level_cam <= black_level_reg;
          cam_buffer_data_waddr <= 0;
        end
      end
    end

    cam_buffer_data_out <= cam_buffer_data[cam_buffer_data_raddr];
    

    

  end

  

  localparam HOST_STATE_IDLE = 0;
  localparam HOST_STATE_REQUEST = 1;
  localparam HOST_STATE_ACKNOWLEDGE = 2;

  reg [7:0] host_state = HOST_STATE_IDLE;

  reg x_ack_host = 0;

  reg [63:0] info_header = 0;
  reg [63:0] info_contents = 0;
  reg [31:0] data_header = 0;

  always @(posedge clk) begin

    x_ack_host <= x_ack;
  
    `mem_begin();

    `reg_32(4 * 1, info_header[31:0]);
    `reg_32(4 * 2, info_header[63:32]);
    `reg_32(4 * 3, info_contents[31:0]);
    `reg_32(4 * 4, info_contents[63:32]);
    `reg_32(4 * 5, data_header);
    `reg_1(4 * 6, enable_packet_tx);
    `reg_32(4 * 7, packet_count);
    `reg_32(4 * 8, black_level_reg);
    
    `reg_1(32'h01000000 + 4 * 0, reg_reset);

    case (host_state) 

      HOST_STATE_IDLE: begin
        if (mem_addr >= 32'h02000000) begin
          x_wstrb <= mem_wstrb;
          x_addr <= mem_addr;
          x_wdata <= mem_wdata;
          host_state <= HOST_STATE_REQUEST;
          mem_error <= 0;
        end
      end

      HOST_STATE_REQUEST: begin
        mem_error <= 0;
        x_req <= 1;
        if (x_ack_host) begin
          host_state <= HOST_STATE_ACKNOWLEDGE;
        end
      end

      HOST_STATE_ACKNOWLEDGE: begin
        mem_error <= 0;
        x_req <= 0;
        if (!x_ack_host) begin
          host_state <= HOST_STATE_IDLE;
          mem_rdata <= x_rdata;
          mem_ready <= 1;
        end
      end

    endcase
    
  end

endmodule
