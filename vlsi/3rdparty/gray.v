// based on
// https://en.wikipedia.org/wiki/Gray_code
// https://raw.githubusercontent.com/cliffordwolf/icotools/master/icosoc/common/icosoc_crossclkfifo.v

function [31:0] binary_to_gray(input [31:0] binary);
    binary_to_gray = binary ^ (binary >> 1);
endfunction

function [31:0] gray_to_binary(input [31:0] gray);
    begin
        gray_to_binary = gray;
        gray_to_binary = gray_to_binary ^ (gray_to_binary >> 16);
        gray_to_binary = gray_to_binary ^ (gray_to_binary >> 8);
        gray_to_binary = gray_to_binary ^ (gray_to_binary >> 4);
        gray_to_binary = gray_to_binary ^ (gray_to_binary >> 2);
        gray_to_binary = gray_to_binary ^ (gray_to_binary >> 1);
    end
endfunction