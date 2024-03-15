// 2023-2024 Philipp Ruppel

#include "verilated.h"
#include "verilated_vcd_c.h"

#include <array>
#include <iostream>
#include <memory>

#include <opencv2/opencv.hpp>
#include <opencv2/quality/qualitypsnr.hpp>

cv::Mat normalizeImage(const cv::Mat &image) {
  cv::Mat img;
  image.copyTo(img);

  cv::Scalar mean, stddev;
  cv::meanStdDev(img, mean, stddev);
  img = (img - mean) / stddev;
  img = img * 0.2 + 0.5;
  return img;
}

void visizalize(const std::string &title, const cv::Mat &image) {
  int image_scale = 50;
  cv::Mat img = normalizeImage(image);
  cv::resize(img, img, cv::Size(), image_scale, image_scale);
  img.convertTo(img, CV_8UC1, 255);
  cv::applyColorMap(img, img, cv::COLORMAP_JET);
  cv::imshow(title, img);
}

template <class Top> void tactest() {

  bool trace = 0;

  std::cerr << "tactile sim" << std::endl;

  Verilated::traceEverOn(trace);

  Top top;

  VerilatedVcdC vcd;

  double dac_voltage_range = 12;

  if (trace) {
    top.trace(&vcd, 0);
    vcd.open("build/tactest.vcd");
  }

  top.eval();

  std::vector<double> tx_trace_voltages;
  tx_trace_voltages.resize(top.dac_channels, 0.0);

  double dt = 1.0 / top.freq_clock;

  double tx_trace_resistance = 30.0 * 1000;
  double tx_trace_capacitance = 100.0 / 1000 / 1000 / 1000 / 1000;

  std::vector<double> rx_trace_currents;
  rx_trace_currents.resize(top.adc_channels, 0.0);

  double rx_trace_capacitance = 100.0 / 1000 / 1000 / 1000 / 1000;
  double adc_reference_voltage_low = 3.3 / 2 - 0.01;
  double adc_reference_voltage_high = 3.3 / 2 + 0.01;
  double adc_feedback_voltage_low = 0;
  double adc_feedback_voltage_high = 3.3;
  double adc_feedback_resistance = 1.0 * 1000 * 1000;

  std::vector<int> adc_bits;
  adc_bits.resize(top.adc_channels, 0);

  std::vector<double> adc_voltages;
  adc_voltages.resize(top.adc_channels, 0.0);

  double sensor_capacitance_min = 0.0 / 1000 / 1000 / 1000 / 1000;
  double sensor_capacitance_max = 10.0 / 1000 / 1000 / 1000 / 1000;
  cv::RNG rng;
  cv::Mat_<double> sensor_capacitance_matrix(top.dac_channels, top.adc_channels,
                                             0.0);
  cv::Mat_<double> sensor_capacitance_matrix_prev(top.dac_channels,
                                                  top.adc_channels, 0.0);
  auto regen = [&]() {
    sensor_capacitance_matrix.copyTo(sensor_capacitance_matrix_prev);
    for (size_t i = 0; i < top.dac_channels; i++) {
      for (size_t j = 0; j < top.adc_channels; j++) {
        sensor_capacitance_matrix(i, j) =
            rng.uniform(0.0, 1.0) * rng.uniform(0.0, 1.0) *
                (sensor_capacitance_max - sensor_capacitance_min) +
            sensor_capacitance_min;
      }
    }
  };
  regen();

  cv::Mat_<double> reconstruction_matrix_i(top.dac_channels, top.adc_channels,
                                           0.0);
  cv::Mat_<double> reconstruction_matrix_q(top.dac_channels, top.adc_channels,
                                           0.0);

  std::vector<int> adc_counters;
  adc_counters.resize(top.adc_channels, 10000);

  size_t cycles = trace ? top.freq_clock / top.freq_base * 5 : top.freq_clock;

  size_t test_cycles = (trace ? cycles : (top.freq_clock / top.freq_swap));

  double voltage_noise = 1;

  double mains_frequency = 70;
  double mains_hum = 1;

  for (size_t t = 0; t < cycles || !trace; t++) {
    if (t % 10000 == 0)
      std::cout << "t " << t % test_cycles * 100.0 / test_cycles << std::endl;

    for (size_t j = 0; j < top.adc_channels; j++) {
      rx_trace_currents[j] = 0;
    }

    for (size_t i = 0; i < top.dac_channels; i++) {

      double dac_voltage =
          top.dac_elements[i] * dac_voltage_range / ((1 << top.dac_bits) - 1);
      double tx_trace_current =
          (dac_voltage - tx_trace_voltages[i]) / tx_trace_resistance;
      double tx_trace_voltage_change =
          tx_trace_current / tx_trace_capacitance * dt;
      tx_trace_voltages[i] += tx_trace_voltage_change;
      top.tx_trace_voltages[i] = tx_trace_voltages[i];

      for (size_t j = 0; j < top.adc_channels; j++) {
        rx_trace_currents[j] +=
            tx_trace_voltage_change * sensor_capacitance_matrix(i, j) / dt;
      }
    }

    for (size_t j = 0; j < top.adc_channels; j++) {
      adc_voltages[j] += rx_trace_currents[j] / rx_trace_capacitance * dt;
    }

    if (top.out_valid) {
      std::cout << "readout " << (int)top.out_dac << " " << (int)top.out_adc
                << " " << (int)top.out_phase << " " << (int32_t)top.out_data
                << std::endl;
      double v = (int32_t)top.out_data;
      v /= 0.0 + top.freq_base + top.freq_step * top.out_dac;

      if (top.out_phase)
        reconstruction_matrix_q(top.out_dac, top.out_adc) = v;
      else
        reconstruction_matrix_i(top.out_dac, top.out_adc) = v;
    }

    double hum =
        mains_hum * std::sin(t * mains_frequency * M_PI * 2 / top.freq_clock);
    for (size_t j = 0; j < top.adc_channels; j++) {
      double v_noise = rng.gaussian(voltage_noise);
      if (adc_voltages[j] < adc_reference_voltage_low + v_noise + hum)
        adc_bits[j] = 1;
      if (adc_voltages[j] > adc_reference_voltage_high + v_noise + hum)
        adc_bits[j] = 0;
      double adc_feedback_voltage =
          (adc_bits[j] ? adc_feedback_voltage_high : adc_feedback_voltage_low);
      double adc_feedback_current =
          (adc_feedback_voltage - adc_voltages[j]) / adc_feedback_resistance;
      adc_voltages[j] += adc_feedback_current / rx_trace_capacitance * dt;

      adc_counters[j] += (adc_bits[j] ? -1 : +1);
      top.adc_feedback_currents[j] = adc_feedback_current;
      top.adc_voltages[j] = adc_voltages[j];
      top.adc_elements[j] = adc_counters[j];
    }

    vcd.dump(t);

    top.clk = 0;
    top.eval();
    top.clk = 1;
    top.eval();

    if (!trace && (t % test_cycles) == 0 &&
        t > top.freq_clock / top.freq_swap) {

      std::cout << sensor_capacitance_matrix << std::endl;
      std::cout << reconstruction_matrix_i << std::endl;
      std::cout << reconstruction_matrix_q << std::endl;
      std::cout << "display" << std::endl;

      visizalize("Capacitance Prev", sensor_capacitance_matrix_prev);
      visizalize("Capacitance", sensor_capacitance_matrix);
      visizalize("Inphase", reconstruction_matrix_i);
      visizalize("Quadrature", reconstruction_matrix_q);

      cv::Mat img = reconstruction_matrix_i.mul(reconstruction_matrix_i) +
                    reconstruction_matrix_q.mul(reconstruction_matrix_q);
      cv::sqrt(img, img);
      visizalize("Magnitude", img);

      cv::Mat a = normalizeImage(sensor_capacitance_matrix);
      cv::Mat b = normalizeImage(img);

      std::cout << "MSE "
                << cv::quality::QualityMSE::compute(a, b, cv::noArray())
                << std::endl;

      std::cout << "PSNR "
                << cv::quality::QualityPSNR::compute(a, b, cv::noArray())
                << std::endl;

      int k = cv::waitKey(0);
      std::cout << k << std::endl;
    }
  }

  std::cout << "ready" << std::endl;

  return;
}