//
//  appV1App.swift
//  appV1
//
//  Created by Ryan Yue on 3/9/25.
//

import SwiftUI
import SwiftData

@main
struct appV1App: App {
    init() {
            print("🚀 App Started Successfully") // ✅ Ensures logs are printing
            UserDefaults.standard.set(true, forKey: "OS_ACTIVITY_MODE") // ✅ Forces debug logging
        }
    var sharedModelContainer: ModelContainer = {
        let schema = Schema([
            Item.self,
        ])
        let modelConfiguration = ModelConfiguration(schema: schema, isStoredInMemoryOnly: false)

        do {
            return try ModelContainer(for: schema, configurations: [modelConfiguration])
        } catch {
            fatalError("Could not create ModelContainer: \(error)")
        }
    }()

    @StateObject var bleManager = BLEManager()  // Shared BLE manager instanc
    
    var body: some Scene {
        WindowGroup {
            TabView {
                MetricsView(bleManager: bleManager)
                        .tabItem {
                            Label("Metrics", systemImage: "heart.text.square")
                        }
                DataLoggingView(bleManager: bleManager)
                    .tabItem {
                        Label("Data Logging", systemImage: "square.and.pencil")
                    }
                DebugLogView(bleManager: bleManager)
                    .tabItem {
                        Label("Debug Log", systemImage: "terminal")
                    }
                
            }
        }
        .modelContainer(sharedModelContainer)
    }
}
