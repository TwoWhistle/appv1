//
//  MetricsView.swift
//  appV1
//
//  Created by Ryan Yue on 3/9/25.
//

import SwiftUI

struct MetricsView: View {
    @ObservedObject var bleManager: BLEManager
    
    var body: some View {
        NavigationView {
            Form {
                BatterySection(bleManager: bleManager)
                VitalSignsSection(bleManager: bleManager)
                ECGFeaturesSection(bleManager: bleManager)
                SCD41SensorSection(bleManager: bleManager)
                BloodPressureSection(bleManager: bleManager)
                EEGBandsSection(bleManager: bleManager)
            }
            .navigationTitle("Live Metrics")
        }
    }
}

// MARK: - 🔹 Battery Section
struct BatterySection: View {
    //let bleManager: BLEManager
    @ObservedObject var bleManager: BLEManager
    
    var body: some View {
        Section(header: Text("Battery Life")) {
            MetricRow(label: "EEG Battery Life", value: bleManager.eegBat, unit: "%", color: .white)
            MetricRow(label: "Wristband Battery Life", value: bleManager.wristBat, unit: "%", color: .white)
        }
    }
}

// MARK: - 🔹 Vital Signs Section
struct VitalSignsSection: View {
    //let bleManager: BLEManager
    @ObservedObject var bleManager: BLEManager
    
    var body: some View {
        Section(header: Text("Vital Signs")) {
            MetricRow(label: "SpO₂", value: bleManager.computedSpO2, unit: "%", color: .blue)
            MetricRow(label: "Heart Rate", value: bleManager.computedHeartRate, unit: "BPM", color: .blue)
            MetricRow(label: "Resp Rate", value: bleManager.computedRespRate, unit: "br/min", color: .blue)
            MetricRow(label: "HRV", value: bleManager.computedHRV, unit: "ms", color: .blue)
            MetricRow(label: "PTT", value: bleManager.computedPTT, unit: "s", color: .blue, specifier: "%.3f")
        }
    }
}

// MARK: - 🔹 ECG Features Section
struct ECGFeaturesSection: View {
    //let bleManager: BLEManager
    @ObservedObject var bleManager: BLEManager
    
    var body: some View {
        Section(header: Text("ECG Features")) {
            let qrsValue = bleManager.computedECGFeatures["QRS"] ?? 0
            let stValue = bleManager.computedECGFeatures["ST"] ?? 0
            MetricRow(label: "QRS Duration", value: qrsValue, unit: "s", color: .green, specifier: "%.3f")
            MetricRow(label: "ST Amplitude", value: stValue, unit: "", color: .green, specifier: "%.3f")
        }
    }
}

// MARK: - 🔹 SCD41 Sensor Section
struct SCD41SensorSection: View {
    //let bleManager: BLEManager
    @ObservedObject var bleManager: BLEManager
    
    var body: some View {
        Section(header: Text("SCD41 Sensor")) {
            MetricRow(label: "CO₂", value: bleManager.lastCO2, unit: "ppm", color: .red)
            MetricRow(label: "Temperature", value: bleManager.lastTemperature, unit: "°C", color: .red, specifier: "%.2f")
            MetricRow(label: "Humidity", value: bleManager.lastHumidity, unit: "%", color: .red, specifier: "%.2f")
        }
    }
}

// MARK: - 🔹 Blood Pressure Section
struct BloodPressureSection: View {
    //let bleManager: BLEManager
    @ObservedObject var bleManager: BLEManager
    
    var body: some View {
        Section(header: Text("Blood Pressure")) {
            MetricRow(label: "Systolic", value: bleManager.computedSystolicBP, unit: "mmHg", color: .purple)
            MetricRow(label: "Diastolic", value: bleManager.computedDiastolicBP, unit: "mmHg", color: .purple)
        }
    }
}

// MARK: - 🔹 EEG Bands Section
struct EEGBandsSection: View {
    //let bleManager: BLEManager
    @ObservedObject var bleManager: BLEManager
    
    var body: some View {
        Section(header: Text("EEG Bands")) {
            let delta = bleManager.computedEegBands["Delta"] ?? 2
            let theta = bleManager.computedEegBands["Theta"] ?? 2
            let alpha = bleManager.computedEegBands["Alpha"] ?? 2
            let beta = bleManager.computedEegBands["Beta"] ?? 2
            let gamma = bleManager.computedEegBands["Gamma"] ?? 2
            MetricRow(label: "Delta", value: delta, unit: "µV²", color: .yellow, specifier: "%.3f")
            MetricRow(label: "Theta", value: theta, unit: "µV²", color: .yellow, specifier: "%.3f")
            MetricRow(label: "Alpha", value: alpha, unit: "µV²", color: .yellow, specifier: "%.3f")
            MetricRow(label: "Beta", value: beta, unit: "µV²", color: .yellow, specifier: "%.3f")
            MetricRow(label: "Gamma", value: gamma, unit: "µV²", color: .yellow, specifier: "%.3f")
        }
    }
}

// MARK: - 🔹 Reusable MetricRow View
struct MetricRow: View {
    let label: String
    let value: Float
    let unit: String
    let color: Color
    var specifier: String = "%.1f"
    
    var body: some View {
        HStack {
            Text(label)
            Spacer()
            Text("\(value, specifier: specifier) \(unit)")
                .foregroundColor(color)
        }
    }
}

// MARK: - 🔹 EEG Band Colors
private func colorForEEGBand(_ band: String) -> Color {
    switch band {
    case "Delta": return .blue
    case "Theta": return .purple
    case "Alpha": return .green
    case "Beta": return .orange
    case "Gamma": return .red
    default: return .black
    }
}

// MARK: - Preview
struct MetricsView_Previews: PreviewProvider {
    static var previews: some View {
        MetricsView(bleManager: BLEManager())
    }
}

