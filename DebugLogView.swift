//
//  DebugLogView.swift
//  appV1
//
//  Created by Ryan Yue on 3/9/25.
//

import SwiftUI

struct DebugLogView: View {
    @ObservedObject var bleManager: BLEManager
    
    var body: some View {
        VStack {
            Text("Debug Log")
                .font(.title)
                .padding()
            
            ScrollView {
                VStack(alignment: .leading) {
                    ForEach(bleManager.debugLogs, id: \..self) { log in
                        Text(log)
                            .font(.system(size: 12))
                            .padding(.vertical, 2)
                            .frame(maxWidth: .infinity, alignment: .leading)
                    }
                }
            }
            .background(Color.black.opacity(0.1))
            .cornerRadius(10)
            .padding()
        }
    }
}
