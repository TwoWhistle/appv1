//
//  BLEManager.swift
//  appV1
//
//  Created by Ryan Yue on 3/9/25.
//

import CoreBluetooth
import SwiftUI
import Accelerate

class BLEManager: NSObject, ObservableObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    
    // MARK: - Published Properties for SwiftUI
    @Published var debugLogs: [String] = []
    
    private var wristChunkBuffer: String = ""
    private var eegChunkBuffer: String = ""
    
    // Computed metrics (wrist data)
    @Published var computedSpO2: Float = 0.0
    @Published var computedHeartRate: Float = 0.0
    @Published var computedRespRate: Float = 0.0
    @Published var computedHRV: Float = 0.0
    @Published var computedECGFeatures: [String: Float] = ["QRS": 0, "ST": 0]
    @Published var computedPTT: Float = 0.0
    @Published var computedSystolicBP: Float = 0.0
    @Published var computedDiastolicBP: Float = 0.0
    
    // SCD41 readouts
    @Published var lastCO2: Float = 0.0
    @Published var lastTemperature: Float = 0.0
    @Published var lastHumidity: Float = 0.0
    
    // For debugging raw chunk
    @Published var lastReceivedChunk: String = ""
    
    // --- EEG Data ---
    // We'll just store raw EEG chunks, or you can parse them further if needed.
    @Published var lastEEGChunk: String = ""     // Most recently received chunk
    @Published var allEEGChunks: [String] = []   // Keep a history of raw EEG data chunks
    @Published var eegBands: [String: Float] = [
        "Delta": 0.0,
        "Theta": 0.0,
        "Alpha": 0.0,
        "Beta": 0.0,
        "Gamma": 0.0
    ]


    // MARK: - BLE References
    private var centralManager: CBCentralManager!
        
    // Wrist
    private var wristPeripheral: CBPeripheral?
    private var wristDataCharacteristic: CBCharacteristic?
        
    // EEG
    private var eegPeripheral: CBPeripheral?
    private var eegDataCharacteristic: CBCharacteristic?
        
    // UUIDs for the Wrist device
    let wristServiceUUID  = CBUUID(string: "6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
    let wristCharUUID     = CBUUID(string: "6E400002-B5A3-F393-E0A9-E50E24DCCA9E")
        
    // UUIDs for the EEG device
    // (Replace with your actual EEG service/characteristic UUIDs if different)
    let eegServiceUUID    = CBUUID(string: "7B36A001-7C44-4D49-B574-2E18D904AF71")
    let eegCharUUID       = CBUUID(string: "7B36A002-7C44-4D49-B574-2E18D904AF71")
        
    // MARK: - Init
    override init() {
        super.init()
        log("🔵 BLEManager Init - Starting Central Manager")
        centralManager = CBCentralManager(delegate: self, queue: nil)
    }
    
    // MARK: - CBCentralManagerDelegate
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        if central.state == .poweredOn {
            log("✅ Bluetooth ON - Scanning for both wrist & EEG services")
            
            // We want to discover both wrist & EEG devices at once.
            // We'll pass both service UUIDs in the array:
            central.scanForPeripherals(withServices: [wristServiceUUID, eegServiceUUID],
                                       options: nil)
        } else {
            log("❌ Bluetooth OFF/Unavailable: \(central.state.rawValue)")
        }
    }
    
    func centralManager(_ central: CBCentralManager,
                        didDiscover peripheral: CBPeripheral,
                        advertisementData: [String : Any],
                        rssi RSSI: NSNumber) {
        
        let name = peripheral.name ?? "Unknown"
        log("🔍 Discovered \(name), RSSI: \(RSSI)")
        
        // Check if it's the wrist device
        if name.lowercased().contains("nrfwr"), wristPeripheral == nil {
            log("✅ Found nrfwrist, connecting...")
            self.wristPeripheral = peripheral
            self.wristPeripheral?.delegate = self
            central.connect(peripheral, options: nil)
        }
        
        // Check if it's the EEG device
        if name.lowercased().contains("nrfear"), eegPeripheral == nil {
            log("✅ Found nrfear (EEG), connecting...")
            self.eegPeripheral = peripheral
            self.eegPeripheral?.delegate = self
            central.connect(peripheral, options: nil)
        }

        // Stop scanning **only if both** devices are found
        if wristPeripheral != nil && eegPeripheral != nil {
            log("✅ Both wrist and EEG devices found, stopping scan.")
            central.stopScan()
        }
    }

    
    func centralManager(_ central: CBCentralManager,
                        didConnect peripheral: CBPeripheral) {
        log("✅ Connected to \(peripheral.name ?? "Unknown")")
        
        // Check which device is connected
        if peripheral == wristPeripheral {
            log("🔍 Discovering wrist services")
            peripheral.discoverServices([wristServiceUUID])
        } else if peripheral == eegPeripheral {
            log("🔍 Discovering EEG services")
            peripheral.discoverServices([eegServiceUUID])
        }
    }
    
    // MARK: - CBPeripheralDelegate
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        if let err = error {
            log("❌ Service discovery error: \(err.localizedDescription)")
            return
        }
        guard let services = peripheral.services else {
            log("❌ No services found.")
            return
        }
        
        for service in services {
            // Wrist device
            if service.uuid == wristServiceUUID {
                log("🔍 Discovering wrist characteristics for \(service.uuid)")
                peripheral.discoverCharacteristics([wristCharUUID], for: service)
            }
            // EEG device
            else if service.uuid == eegServiceUUID {
                log("🔍 Discovering EEG characteristics for \(service.uuid)")
                peripheral.discoverCharacteristics([eegCharUUID], for: service)
            }
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral,
                    didDiscoverCharacteristicsFor service: CBService,
                    error: Error?) {
        if let err = error {
            log("❌ Characteristic discovery error: \(err.localizedDescription)")
            return
        }
        guard let characteristics = service.characteristics else {
            log("❌ No characteristics found.")
            return
        }
        
        for characteristic in characteristics {
            // Wrist data characteristic
            if service.uuid == wristServiceUUID &&
               characteristic.uuid == wristCharUUID {
                wristDataCharacteristic = characteristic
                log("✅ Found Wrist Data Characteristic. Enabling notifications...")
                peripheral.setNotifyValue(true, for: characteristic)
                log("✅ Notifications Enabled for Wrist. Waiting for wrist data...")
            }
            // EEG data characteristic
            else if service.uuid == eegServiceUUID &&
                    characteristic.uuid == eegCharUUID {
                eegDataCharacteristic = characteristic
                log("✅ Found EEG Data Characteristic. Enabling notifications...")
                peripheral.setNotifyValue(true, for: characteristic)
                log("✅ Notifications Enabled for EEG. Waiting for eeg data...")
            }
        }
    }
    
    // MARK: - Receiving Chunks
    func peripheral(_ peripheral: CBPeripheral,
                    didUpdateValueFor characteristic: CBCharacteristic,
                    error: Error?) {
        if let err = error {
            log("❌ Error receiving data: \(err.localizedDescription)")
            return
        }
        
        guard let data = characteristic.value,
              let partialString = String(data: data, encoding: .utf8)
        else {
            log("❌ No Data or invalid UTF-8 encoding")
            return
        }
        
        // === WRIST DATA? ===
        if characteristic == wristDataCharacteristic {
            log("📡 Received Wrist Chunk: \(partialString)")
            // Accumulate into wristChunkBuffer
            wristChunkBuffer.append(partialString)
            
            // Look for one or more '*' markers
            while let endMarkerRange = wristChunkBuffer.range(of: "*") {
                // Extract everything up to '*'
                let completeChunk = String(wristChunkBuffer[..<endMarkerRange.lowerBound])
                log("final wrist chunk found")
                
                // Remove it from the buffer (including '*')
                let removeUpToIndex = endMarkerRange.upperBound
                wristChunkBuffer.removeSubrange(..<removeUpToIndex)
                
                // Parse that complete chunk
                log("parsing full wrist chunk")
                handleWristCompleteChunk(completeChunk)
                log("full wrist chunk processed")
            }
        }
        // === EEG DATA? ===
        else if characteristic == eegDataCharacteristic {
            log("📡 Received EEG Chunk: \(partialString)")
            // Accumulate into eegChunkBuffer
            eegChunkBuffer.append(partialString)
            
            // Look for '*' end markers
            while let endMarkerRange = eegChunkBuffer.range(of: "*") {
                let completeChunk = String(eegChunkBuffer[..<endMarkerRange.lowerBound])
                log("final eeg chunk found")
                
                let removeUpToIndex = endMarkerRange.upperBound
                eegChunkBuffer.removeSubrange(..<removeUpToIndex)
                
                log("parsing full eeg chunk")
                handleEEGCompleteChunk(completeChunk)
                log("full eeg chunk processed")
            }
        }
    }
    
    // MARK: - Wrist Parsing
    private func handleWristCompleteChunk(_ rawString: String) {
        // For debug, store it in lastReceivedChunk
        lastReceivedChunk = rawString
        log("📡 Full Wrist Chunk: \(rawString)")

        // Split into 4 parts by ';'
        let parts = rawString.components(separatedBy: ";")
        guard parts.count == 4 else {
            log("⚠️ Incorrect chunk format (need 4 sections).")
            return
        }
        
        let timestampStr = parts[0]  // e.g. "123456"
        let ecgPart      = parts[1]  // e.g. "ECG,512,514,..."
        let ppgPart      = parts[2]  // e.g. "PPG,1024,1022,..."
        let scdPart      = parts[3]  // e.g. "SCD,435.12,25.76,48.2"

        // Convert timestamp if needed
        let _ = Float(timestampStr) ?? 0.0
        
        // ========== PARSE ECG SAMPLES ==========
        var ecgSamples: [Float] = []
        let ecgComponents = ecgPart.components(separatedBy: ",")
        if ecgComponents.count > 1, ecgComponents[0] == "ECG" {
            for val in ecgComponents.dropFirst() {
                if let f = Float(val) {
                    ecgSamples.append(f)
                }
            }
        }

        // ========== PARSE PPG SAMPLES ==========
        var ppgSamples: [Float] = []
        let ppgComponents = ppgPart.components(separatedBy: ",")
        if ppgComponents.count > 1, ppgComponents[0] == "PPG" {
            for val in ppgComponents.dropFirst() {
                if let f = Float(val) {
                    ppgSamples.append(f)
                }
            }
        }

        // ========== PARSE SCD41 DATA ==========
        // Format: "SCD,co2,temp,hum"
        let scdComponents = scdPart.components(separatedBy: ",")
        if scdComponents.count == 4, scdComponents[0] == "SCD" {
            if let co2Val = Float(scdComponents[1]),
               let tempVal = Float(scdComponents[2]),
               let humVal  = Float(scdComponents[3]) {
               
                DispatchQueue.main.async {
                    self.lastCO2         = co2Val
                    self.lastTemperature = tempVal
                    self.lastHumidity    = humVal
                }
            } else {
                log("⚠️ SCD parse error: co2/temp/hum not floats.")
            }
        } else {
            log("⚠️ SCD format mismatch: \(scdPart)")
        }

        // ========== ADVANCED PROCESSING ==========
        let (filteredECG, filteredPPG) = runAdvancedProcessing(ecgSamples: ecgSamples,
                                                               ppgSamples: ppgSamples)
        let results = computeBiometrics(filteredECG: filteredECG, filteredPPG: filteredPPG)
        
        // Compute blood pressure based on PTT (this is a naive placeholder formula)
        let bp = computeBloodPressure(ptt: results.ptt)
        
        DispatchQueue.main.async {
            self.computedSpO2      = results.spo2
            self.computedHeartRate = results.hr
            self.computedRespRate  = results.respRate
            self.computedHRV       = results.hrv
            self.computedECGFeatures = results.ecgFeatures
            self.computedPTT       = results.ptt
            self.computedSystolicBP = bp.systolic
            self.computedDiastolicBP = bp.diastolic
        }
        
        saveWristData(
            rawChunk: rawString,
            spo2: self.computedSpO2,
            hr: self.computedHeartRate,
            respRate: self.computedRespRate,
            hrv: self.computedHRV,
            ecgFeatures: self.computedECGFeatures,
            ptt: self.computedPTT,
            systolicBP: self.computedSystolicBP,
            diastolicBP: self.computedDiastolicBP
        )
    }
    
    // MARK: - EEG Parsing
    private func handleEEGCompleteChunk(_ rawString: String) {
        log("📡 Final EEG Chunk: \(rawString)")
        
        // Example: raw EEG chunk might be "512,513,511,517,..." or any format
        // If you just want to store it as-is, do:
        
        lastEEGChunk = rawString
        allEEGChunks.append(rawString)
        
        // 1) Parse rawString into an array of Float samples
        let samplesStr = rawString.components(separatedBy: ",")
        var eegSamples: [Float] = []
        for s in samplesStr {
            if let val = Float(s.trimmingCharacters(in: .whitespacesAndNewlines)) {
                eegSamples.append(val)
            }
        }
        
        // 2) If we have enough samples, compute basic band powers
        if eegSamples.count > 8 {  // Need a minimum for FFT
            let newBands = computeEEGBands(eegSamples)
            
            // 3) Update UI on main thread
            DispatchQueue.main.async {
                self.eegBands = newBands
            }
        }
        
        saveEEGData(rawString)
    }
    
    // MARK: - Disconnect Handling
    func centralManager(_ central: CBCentralManager,
                        didDisconnectPeripheral peripheral: CBPeripheral,
                        error: Error?) {
        let name = peripheral.name ?? "Unknown"
        log("❌ Disconnected from \(name)")
        
        // If you want to reconnect automatically, do so here
        // Or re-scan for both devices
        central.scanForPeripherals(withServices: [wristServiceUUID, eegServiceUUID], options: nil)
    }
    
    

    
    // MARK: - Signal Processing (Replace in backend)
    //this funationality would be replaced in the backend where it would include more advanced denoising and processing
    private func runAdvancedProcessing(ecgSamples: [Float],
                                       ppgSamples: [Float])
    -> (filteredECG: [Float], filteredPPG: [Float])
    {
        let denoisedECG = denoiseSignal(ecgSamples)
        let denoisedPPG = denoiseSignal(ppgSamples)
        return (denoisedECG, denoisedPPG)
    }
    
    private func denoiseSignal(_ signal: [Float]) -> [Float] {
        return movingAverageFilter(bandpassFilterLegacy(signal), windowSize: 5)
    }
    
    private func movingAverageFilter(_ signal: [Float], windowSize: Int = 5) -> [Float] {
        guard signal.count > windowSize else { return signal }
        var output = [Float](repeating: 0.0, count: signal.count)
        for i in 0..<(signal.count - windowSize) {
            var sum: Float = 0
            for j in 0..<windowSize {
                sum += signal[i + j]
            }
            output[i + windowSize/2] = sum / Float(windowSize)
        }
        return output
    }
    
    private func biquadFilter(
        _ signal: [Float],
        b0: Float,
        b1: Float,
        b2: Float,
        a1: Float,
        a2: Float
    ) -> [Float] {
        var x1: Float = 0, x2: Float = 0
        var output = [Float](repeating: 0.0, count: signal.count)
        for i in 0..<signal.count {
            let w = signal[i] - a1*x1 - a2*x2
            let y = b0*w + b1*x1 + b2*x2
            x2 = x1
            x1 = w
            output[i] = y
        }
        return output
    }
    
    private func bandpassFilterLegacy(_ signal: [Float]) -> [Float] {
        guard signal.count > 2 else { return signal }
        let b0: Float =  0.095465
        let b1: Float =  0.0
        let b2: Float = -0.095465
        let a1: Float = -1.808
        let a2: Float =  0.80907
        return biquadFilter(signal, b0: b0, b1: b1, b2: b2, a1: a1, a2: a2)
    }
    
    
    // MARK: - COMPUTE BIOMETRICS (Replace in backend)
    //this funationality would be replaced in the backend where it would include more advanced computing and inferencing
    
    private func computeBiometrics(filteredECG: [Float],
                                       filteredPPG: [Float])
        -> (spo2: Float, hr: Float, respRate: Float, hrv: Float,
            ecgFeatures: [String: Float], ptt: Float)
        {
            // SpO2
            let spo2Val = computeSpO2FromPPG(filteredPPG)
            
            // Heart Rate
            let hrVal = computeHeartRate(filteredECG, filteredPPG)
            
            // RR via FFT approach on ECG
            let respRateVal = computeRespiratoryRate(filteredECG, filteredPPG)
            
            // HRV (RMSSD) using the same R-peaks from Pan–Tompkins
            let hrvVal = computeHRV_PanTompkins(filteredECG)
            
            // ECG Features
            let ecgFeats = computeECGFeaturesNaive(ecg: filteredECG)
            // PTT remains the same or can do advanced approach:
            let pttVal = computePTT(filteredECG, filteredPPG)
            
            return (spo2Val, hrVal, respRateVal, hrvVal, ecgFeats, pttVal)
        }
    
    private func computeEEGBands(_ eeg: [Float]) -> [String: Float] {
        // Adjust the sampling rate to match your hardware
        let sampleRate: Float = 100.0

        // 1) Zero-pad eeg[] to next power of two for FFT
        let n = eeg.count
        let log2n = vDSP_Length(floor(log2(Double(n))))
        let fftSize = Int(1 << log2n)
        // If we have leftover samples, we can either drop or pad
        var signal = (fftSize < n)
            ? Array(eeg[0..<fftSize])
            : eeg + Array(repeating: 0.0, count: fftSize - n)

        // 2) Create interleaved complex array
        var interleaved = [Float](repeating: 0.0, count: fftSize * 2)
        for i in 0..<fftSize {
            interleaved[2 * i]     = signal[i]
            interleaved[2 * i + 1] = 0.0
        }

        // 3) Perform FFT using Accelerate
        var realp = [Float](repeating: 0.0, count: fftSize/2)
        var imagp = [Float](repeating: 0.0, count: fftSize/2)
        var splitComplex = DSPSplitComplex(realp: &realp, imagp: &imagp)

        guard let setup = vDSP_create_fftsetup(vDSP_Length(log2n), FFTRadix(kFFTRadix2)) else {
            return ["Delta": 0, "Theta": 0, "Alpha": 0, "Beta": 0, "Gamma": 0]
        }

        interleaved.withUnsafeMutableBytes { ptr in
            let typedPtr = ptr.baseAddress!.bindMemory(to: DSPComplex.self, capacity: fftSize)
            vDSP_ctoz(typedPtr, 2, &splitComplex, 1, vDSP_Length(fftSize/2))
        }
        vDSP_fft_zrip(setup, &splitComplex, 1, vDSP_Length(log2n), FFTDirection(FFT_FORWARD))
        vDSP_destroy_fftsetup(setup)

        // 4) Compute magnitude squared => power spectrum
        var powerSpectrum = [Float](repeating: 0.0, count: fftSize/2)
        vDSP_zvabs(&splitComplex, 1, &powerSpectrum, 1, vDSP_Length(fftSize/2))
        // We have amplitude; square it to get power
        powerSpectrum = powerSpectrum.map { $0 * $0 }

        // 5) Sum power in each frequency band
        // The frequency resolution is sampleRate / fftSize
        let freqResolution = sampleRate / Float(fftSize)

        func bandPower(_ lowFreq: Float, _ highFreq: Float) -> Float {
            let lowIndex  = Int(floor(lowFreq / freqResolution))
            let highIndex = Int(floor(highFreq / freqResolution))
            if highIndex > powerSpectrum.count - 1 {
                return 0.0
            }
            if lowIndex >= powerSpectrum.count || lowIndex >= highIndex {
                return 0.0
            }
            let subrange = powerSpectrum[lowIndex..<min(highIndex, powerSpectrum.count)]
            return subrange.reduce(0, +)
        }

        let delta  = bandPower(0.5, 4.0)
        let theta  = bandPower(4.0, 8.0)
        let alpha  = bandPower(8.0, 12.0)
        let beta   = bandPower(12.0, 30.0)
        let gamma  = bandPower(30.0, 50.0)

        return [
            "Delta": delta,
            "Theta": theta,
            "Alpha": alpha,
            "Beta":  beta,
            "Gamma": gamma
        ]
    }

    
    private func computePTT(_ ecg: [Float], _ ppg: [Float]) -> Float {
           // 1) Find the first major R-peak in ECG
           guard let ecgMax = ecg.max(), ecgMax > 0 else { return 0.25 }
           let thresholdECG = ecgMax * 0.6
           var rPeakIndex: Int?
           for i in 1..<(ecg.count - 1) {
               if ecg[i] > thresholdECG && ecg[i] > ecg[i-1] && ecg[i] > ecg[i+1] {
                   rPeakIndex = i
                   break
               }
           }
           guard let rIndex = rPeakIndex else { return 0.25 }
           
           // 2) Find the foot (onset) of the PPG waveform after that R-peak
           // We'll look for a local minimum in PPG after rIndex
           let searchRange = (rIndex+1)..<ppg.count
           guard searchRange.count > 2 else { return 0.25 }
           
           var minVal: Float = ppg[rIndex+1]
           var footIndex: Int = rIndex+1
           for i in (rIndex+1)..<ppg.count - 1 {
               if ppg[i] < minVal && ppg[i] < ppg[i+1] {
                   minVal = ppg[i]
                   footIndex = i
               }
           }
           
           // 3) Compute time difference in seconds
           let samplingRate: Float = 100.0
           let deltaSamples = Float(footIndex - rIndex)
           let pttSeconds = deltaSamples / samplingRate
           
           return pttSeconds
       }


    /// Naively estimate blood pressure based on pulse transit time (PTT).
    /// This is a placeholder and should be replaced with a clinically validated method.
    private func computeBloodPressure(ptt: Float) -> (systolic: Float, diastolic: Float) {
        // Assume: At a PTT of 0.25 seconds, blood pressure is around 120/80.
        // For every 0.01 s deviation from 0.25, adjust:
        // - Systolic changes by 0.5 mmHg (example: higher PTT means lower pressure)
        // - Diastolic changes by 0.3 mmHg
        let basePTT: Float = 0.25
        let systolicBase: Float = 120.0
        let diastolicBase: Float = 80.0
        let systolicAdjustment: Float = 50.0  // example scaling factor
        let diastolicAdjustment: Float = 30.0
        
        let systolic = systolicBase - (ptt - basePTT) * systolicAdjustment
        let diastolic = diastolicBase - (ptt - basePTT) * diastolicAdjustment
        
        // Clamp values to a realistic range:
        let clampedSystolic = max(90.0, min(systolic, 180.0))
        let clampedDiastolic = max(60.0, min(diastolic, 120.0))
        
        return (clampedSystolic, clampedDiastolic)
    }
    
    private func computeSpO2FromPPG(_ ppg: [Float]) -> Float {
        // Ensure we have enough samples.
        guard ppg.count > 10 else { return 98.0 }
        
        // Compute the DC component (average) and the AC component (peak-to-peak)
        let dc = ppg.reduce(0, +) / Float(ppg.count)
        let ac = (ppg.max() ?? dc) - (ppg.min() ?? dc)
        
        // Compute ratio of AC to DC
        let ratio = ac / dc
        
        // Empirical formula (example): adjust constants as needed.
        // For instance, if ratio = 0.1, then spo2 ≈ 110 - 25 * 0.1 = 107.5 (clamped to 100)
        // If ratio = 0.3, then spo2 ≈ 110 - 25 * 0.3 = 102.5 (clamped to 100)
        // So for a higher ratio, you get lower estimated SpO₂.
        let spo2Estimate = 110.0 - 25.0 * ratio
        //let bounded = max(90.0, min(spo2Estimate, 100.0))
        //return bounded
        return spo2Estimate
    }

    private func computeHeartRate(_ ecg: [Float], _ ppg: [Float]) -> Float {
        guard ecg.count > 10 else { return 70.0 }
        
        // Use a lower threshold relative to the maximum value to be more sensitive.
        let threshold: Float = (ecg.max() ?? 0) * 0.5
        var peakIndices: [Int] = []
        
        // Set a refractory period of 150ms at 100Hz (i.e., 15 samples)
        let refractoryPeriod = Int(0.15 * 100.0)
        var lastPeakIndex = -refractoryPeriod
        
        for i in 1..<(ecg.count - 1) {
            if ecg[i] > threshold &&
                ecg[i] > ecg[i - 1] &&
                ecg[i] > ecg[i + 1] &&
                (i - lastPeakIndex) >= refractoryPeriod {
                
                peakIndices.append(i)
                lastPeakIndex = i
            }
        }
        
        // If not enough peaks were detected, return a default value.
        guard peakIndices.count > 1 else { return 70.0 }
        
        // Compute the intervals (in samples) between consecutive peaks.
        var intervals: [Float] = []
        for i in 1..<peakIndices.count {
            intervals.append(Float(peakIndices[i] - peakIndices[i - 1]))
        }
        
        // Average the intervals.
        let avgInterval = intervals.reduce(0, +) / Float(intervals.count)
        
        // Convert sample interval to time (seconds) and then to BPM.
        let samplingRate: Float = 100.0
        let avgRRsec = avgInterval / samplingRate
        let hr = 60.0 / avgRRsec
        
        //return max(40.0, min(hr, 180.0))
        return hr
    }


    
    private func computeRespiratoryRate(_ ecg: [Float], _ ppg: [Float]) -> Float {
        let n = ecg.count
        guard n > 1 else { return 12.0 }
        let log2n = vDSP_Length(floor(log2(Double(n))))
        let fftSize = Int(1 << log2n)
        guard fftSize > 0 else { return 12.0 }
        var ecgDouble = ecg.map { Double($0) }
        if fftSize < ecgDouble.count {
            ecgDouble = Array(ecgDouble[0..<fftSize])
        } else if fftSize > ecgDouble.count {
            ecgDouble += [Double](repeating: 0.0, count: fftSize - ecgDouble.count)
        }
        var interleaved = [Double](repeating: 0.0, count: fftSize * 2)
        for i in 0..<fftSize {
            interleaved[2 * i]     = ecgDouble[i]
            interleaved[2 * i + 1] = 0.0
        }
        var realp = [Double](repeating: 0.0, count: fftSize/2)
        var imagp = [Double](repeating: 0.0, count: fftSize/2)
        var splitComplex = DSPDoubleSplitComplex(realp: &realp, imagp: &imagp)
        guard let setup = vDSP_create_fftsetupD(log2n, FFTRadix(kFFTRadix2)) else {
            return 12.0
        }
        interleaved.withUnsafeBufferPointer { ptr in
            ptr.baseAddress!.withMemoryRebound(to: DSPDoubleComplex.self, capacity: fftSize) { typedPtr in
                vDSP_ctozD(typedPtr, 2, &splitComplex, 1, vDSP_Length(fftSize/2))
            }
        }
        vDSP_fft_zripD(setup, &splitComplex, 1, log2n, FFTDirection(FFT_FORWARD))
        var magnitudes = [Double](repeating: 0.0, count: fftSize/2)
        vDSP_zvabsD(&splitComplex, 1, &magnitudes, 1, vDSP_Length(fftSize/2))
        vDSP_destroy_fftsetupD(setup)
        let sampleRate = 100.0
        let freqResolution = sampleRate / Double(fftSize)
        let minIndex = Int(ceil(0.1 / freqResolution))
        let maxIndex = Int(floor(0.4 / freqResolution))
        guard minIndex < magnitudes.count,
              maxIndex < magnitudes.count,
              maxIndex > minIndex
        else {
            return 12.0
        }
        let subrange = magnitudes[minIndex...maxIndex]
        guard let maxMag = subrange.max(),
              let maxI = subrange.firstIndex(of: maxMag)
        else {
            return 12.0
        }
        let peakFreq = Double(maxI + minIndex) * freqResolution
        let rrBpm = peakFreq * 60.0
        return Float(rrBpm)
    }
    
    private func computeHRV(_ ecg: [Float]) -> Float {
        guard ecg.count > 10 else { return 40.0 }
        let threshold: Float = (ecg.max() ?? 0) * 0.6
        var peakIndices: [Int] = []
        for i in 1..<(ecg.count - 1) {
            if ecg[i] > threshold && ecg[i] > ecg[i-1] && ecg[i] > ecg[i+1] {
                peakIndices.append(i)
            }
        }
        guard peakIndices.count > 2 else { return 40.0 }
        let samplingRate: Float = 100.0
        var rrIntervalsSec: [Float] = []
        for i in 1..<peakIndices.count {
            let deltaSamples = Float(peakIndices[i] - peakIndices[i-1])
            let deltaSec = deltaSamples / samplingRate
            rrIntervalsSec.append(deltaSec)
        }
        var successiveDiffs: [Float] = []
        for i in 1..<rrIntervalsSec.count {
            let diff = rrIntervalsSec[i] - rrIntervalsSec[i-1]
            successiveDiffs.append(diff * diff)
        }
        guard successiveDiffs.count > 0 else { return 40.0 }
        let meanDiff = successiveDiffs.reduce(0, +) / Float(successiveDiffs.count)
        let rmssd = sqrt(meanDiff) * 1000.0
        return rmssd
    }
    
    private func computeHRV_PanTompkins(_ ecg: [Float]) -> Float {
        let sampleRate: Float = 100.0
        let peaks = panTompkinsPeakDetection(ecg, sampleRate: sampleRate)
        guard peaks.count > 2 else { return 40.0 }
        var rrIntervals: [Float] = []
        for i in 1..<peaks.count {
            let delta = Float(peaks[i] - peaks[i-1]) / sampleRate
            rrIntervals.append(delta)
        }
        var diffsSq: [Float] = []
        for i in 1..<rrIntervals.count {
            let d = rrIntervals[i] - rrIntervals[i-1]
            diffsSq.append(d*d)
        }
        guard diffsSq.count > 0 else { return 40.0 }
        let meanDiff = diffsSq.reduce(0, +) / Float(diffsSq.count)
        let rmssd = sqrt(meanDiff) * 1000.0
        return rmssd
    }
    
    private func bandpassFilter5to15HzLegacy(_ signal: [Float]) -> [Float] {
        guard signal.count > 2 else { return signal }
        let b0: Float =  0.083191
        let b1: Float =  0.0
        let b2: Float = -0.083191
        let a1: Float = -1.729012
        let a2: Float =  0.833618
        return biquadFilter(signal, b0: b0, b1: b1, b2: b2, a1: a1, a2: a2)
    }
    
    private func panTompkinsPeakDetection(_ ecg: [Float],
                                          sampleRate: Float = 100.0) -> [Int] {
        let bandpassed = bandpassFilter5to15HzLegacy(ecg)
        var derivative = [Float](repeating: 0, count: bandpassed.count)
        for i in 2..<(bandpassed.count - 2) {
            derivative[i] = (2*bandpassed[i+1] + bandpassed[i] - bandpassed[i-1] - 2*bandpassed[i-2]) / 8.0
        }
        let squared = derivative.map { $0 * $0 }
        let windowSize = Int(0.15 * sampleRate)
        var integrated = [Float](repeating: 0.0, count: squared.count)
        for i in 0..<(squared.count - windowSize) {
            var sum: Float = 0
            for j in 0..<windowSize {
                sum += squared[i+j]
            }
            integrated[i + windowSize/2] = sum / Float(windowSize)
        }
        let maxVal = integrated.max() ?? 0
        var threshold: Float = maxVal * 0.3
        var signalLevel: Float = threshold
        var noiseLevel: Float  = threshold * 0.5
        let refractory = Int(0.2 * sampleRate)
        var lastPeak = -refractory
        var rPeaks: [Int] = []
        for i in 0..<integrated.count {
            let val = integrated[i]
            if val > threshold && (i - lastPeak) > refractory {
                rPeaks.append(i)
                lastPeak = i
                signalLevel = 0.125 * val + 0.875 * signalLevel
            } else {
                noiseLevel = 0.125 * val + 0.875 * noiseLevel
            }
            threshold = noiseLevel + 0.25 * (signalLevel - noiseLevel)
        }
        return rPeaks
    }
    
    private func findQandSWaves(ecg: [Float],
                                rIndex: Int,
                                fs: Float) -> (Int, Int, Float, Float) {
        let rangeSamples = Int(0.04 * fs)
        let startQ = max(rIndex - rangeSamples, 0)
        let endQ   = rIndex
        var qIndex = rIndex
        var qValue = ecg[rIndex]
        for i in startQ..<endQ {
            if ecg[i] < qValue {
                qValue = ecg[i]
                qIndex = i
            }
        }
        let startS = rIndex
        let endS   = min(rIndex + rangeSamples, ecg.count - 1)
        var sIndex = rIndex
        var sValue = ecg[rIndex]
        for i in startS...endS {
            if ecg[i] < sValue {
                sValue = ecg[i]
                sIndex = i
            }
        }
        return (qIndex, sIndex, qValue, sValue)
    }
    
    private func computeECGFeatures(ecg: [Float],
                                    sampleRate: Float = 100.0)
    -> (avgQRS: Float, avgST: Float) {
        let rPeaks = panTompkinsPeakDetection(ecg, sampleRate: sampleRate)
        guard !rPeaks.isEmpty else { return (0.0, 0.0) }
        var qrsDurations: [Float] = []
        var stAmplitudes: [Float] = []
        for rIndex in rPeaks {
            let (qIndex, sIndex, _, _) = findQandSWaves(ecg: ecg, rIndex: rIndex, fs: sampleRate)
            let qrs = Float(sIndex - qIndex) / sampleRate
            qrsDurations.append(qrs)
            let stOffsetSamples = Int(0.08 * sampleRate)
            let stIndex = sIndex + stOffsetSamples
            if stIndex < ecg.count {
                let stVal = ecg[stIndex]
                stAmplitudes.append(stVal)
            }
        }
        let avgQRS = qrsDurations.isEmpty ? 0.0 : (qrsDurations.reduce(0, +) / Float(qrsDurations.count))
        let avgST = stAmplitudes.isEmpty ? 0.0 : (stAmplitudes.reduce(0, +) / Float(stAmplitudes.count))
        return (avgQRS, avgST)
    }
    
    private func computeECGFeaturesNaive(ecg: [Float]) -> [String: Float] {
        let (avgQRS, avgST) = computeECGFeatures(ecg: ecg, sampleRate: 100.0)
        return [
            "QRS": avgQRS,
            "ST" : avgST
        ]
    }
    
    
    // MARK: - Saving and Exporting
    //also should be repurposed to save and export to the backend database
    
    private func saveWristData(rawChunk: String,
                          spo2: Float,
                          hr: Float,
                          respRate: Float,
                          hrv: Float,
                          ecgFeatures: [String: Float],
                          ptt: Float,
                          systolicBP: Float,
                          diastolicBP: Float)
    {
        let timestamp = Date()
        var ecgFeatureString = ""
        for (key, val) in ecgFeatures {
            ecgFeatureString += "\(key)=\(val);"
        }
        if ecgFeatureString.hasSuffix(";") {
            ecgFeatureString.removeLast()
        }
        
        // Updated CSV format now includes systolic and diastolic blood pressure
        let entry =
        """
        \(timestamp),\(rawChunk),\(spo2),\(hr),\(respRate),\(hrv),\(ecgFeatureString),\(ptt),\(systolicBP),\(diastolicBP)\n
        """
        
        let fileURL = FileManager.default.temporaryDirectory.appendingPathComponent("WristbandDataLog.txt")
        
        if let handle = try? FileHandle(forWritingTo: fileURL) {
            handle.seekToEndOfFile()
            if let data = entry.data(using: .utf8) {
                handle.write(data)
            }
            handle.closeFile()
        } else {
            try? entry.write(to: fileURL, atomically: true, encoding: .utf8)
        }
    }
    
    private func saveEEGData(_ rawString: String) {
        let fileURL = FileManager.default.temporaryDirectory.appendingPathComponent("EEGDataLog.txt")

        // Read existing file content (if any)
        var existingData = ""
        if let existingContent = try? String(contentsOf: fileURL, encoding: .utf8) {
            existingData = existingContent
        }

        // Append the new EEG chunk with a semicolon separator
        let newData = existingData.isEmpty ? rawString : existingData + ";" + rawString

        do {
            try newData.write(to: fileURL, atomically: true, encoding: .utf8)
        } catch {
            log("❌ Error saving EEG data: \(error.localizedDescription)")
        }
    }

    
    func exportWristData() {
        let fileURL = FileManager.default.temporaryDirectory.appendingPathComponent("WristbandDataLog.txt")
        let activityVC = UIActivityViewController(activityItems: [fileURL], applicationActivities: nil)
        UIApplication.shared.windows.first?.rootViewController?.present(activityVC, animated: true)
    }
    
    func exportEEGData() {
        let fileURL = FileManager.default.temporaryDirectory.appendingPathComponent("EEGDataLog.txt")
        let activityVC = UIActivityViewController(activityItems: [fileURL], applicationActivities: nil)
        UIApplication.shared.windows.first?.rootViewController?.present(activityVC, animated: true)
    }
    
    
    // MARK: - Logging Utility
    private func log(_ message: String) {
        print(message)
        DispatchQueue.main.async {
            self.debugLogs.append(message)
            if self.debugLogs.count > 100 {
                self.debugLogs.removeFirst()
            }
        }
    }
}

