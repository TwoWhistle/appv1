//
//  DataLoggingView.swift
//  appV1
//
//  Created by Ryan Yue on 3/9/25.
//

import SwiftUI

struct DataLoggingView: View {
    @ObservedObject var bleManager: BLEManager
    @State private var selectedState: String = ""
    @State private var selectedTime: Date = Date()
    @State private var loggedData: [(Date, String)] = []
        
    var body: some View {
        VStack {
            Text("Data Logging")
                .font(.title)
                .padding()
            
            .padding()
            Button("Export Wrist Data") {
                bleManager.exportWristData()
            }
            .padding()
            .buttonStyle(.borderedProminent)
            Button("Export EEG Data") {
                bleManager.exportEEGData()
            }
            .padding()
            .buttonStyle(.borderedProminent)
        }
    }
}
