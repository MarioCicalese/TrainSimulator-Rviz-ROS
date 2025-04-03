# 🚆 Train Simulation with ROS and RViz

## 🔹 Introduction

This project simulates trains movements using ROS and RViz. The system processes train schedule data, identifies delayed and canceled trains, and visually represents their movement in real-time. The simulation displays stations and trains using markers with different colors based on their status.

## 🌟 Features

- 📊 **Train Data Processing:** Retrieves and processes scheduled train data.
- 🎥 **Real-Time Visualization:** Simulates movement on RViz.
- 🔄 **Custom ROS Topics & Services:** Implements publishers, subscribers, and custom ROS messages.
- 🕹️ **Dynamic Interaction:** Displays real-time trains positions, including delays and cancellations.

## 🛠️ Prerequisites

Ensure you have the following installed:

- 🖥️ **Ubuntu (Recommended: 20.04 or later)**
- 🤖 **ROS (Noetic recommended)**
- 👀 **RViz**
- 🐍 **Python 3**
- 📦 **Pandas library** (`pip install pandas`)

## ⚙️ Public Parameters

- ⏱️ **update_frequency (default: 60)** - The rate at which data is updated.
- ⏳ **delay_threshold (default: 5)** - The number of minutes of delay to consider a train as delayed.

## 📨 Custom Messages

The project uses a custom message type `TrainData.msg`, containing:

- `departure_station` - Train departure location.
- `arrival_destination` - Train destination.
- `date_of_journey` - Date of travel.
- `departure_time` - Scheduled departure time.
- `arrival_time` - Scheduled arrival time.
- `actual_arrival_time` - Real arrival time (if the status is "Delayed").
- `journey_status` - Status of the train (✅ On Time, ⏳ Delayed, ❌ Cancelled).
- `reason_for_delay` - Explanation for delay or cancellation (null if the train is On time).

## 🔊 ROS Topics

- 📢 **`/train_data` (`TrainData.msg`)**: Publishes all trains scheduled for the day.
- 📢 **`/delay_alerts` (`TrainData.msg`)**: Publishes delayed or canceled trains.
- 📍 **`/stations` (`Marker`)**: Publishes station markers.
- 🚆 **`/trains` (`Marker`)**: Publishes markers and movement.

## 🔧 Project Components

### 1️⃣ Retrieving Scheduled trains 🚆

- **Service:** `trainDataService.py` - Returns trains based on current time.
- **Publisher:** `dataCollector.py` - Calls `trainDataService.py` and publishes to `/train_data`.
- **Subscriber:** `scheduleDisplay.py` - Reads and displays `/train_data`.

### 2️⃣ Identifying Delayed & Canceled trains 🚆

- **Service:** `delayAnalysisService.py` - Returns delayed/canceled trains.
- **Publisher:** `delayAnalysisClient.py` - Calls `delayAnalysisService.py` and publishes to `/delay_alerts`.
- **Subscriber:** `scheduleDisplay.py` - Reads and displays `/delay_alerts`.

### 3️⃣ Searching trains by Departure Station 🏙️

- **Service:** `searchTrainService.py` - Returns trains departing from a given departure station.
- **Client:** `searchTrainClient.py` - Calls `searchTrainService.py` and prints results.

### 4️⃣ Adding New trains to Dataset 🚆

- **Service:** `addTrainService.py` - Adds new trains data.
- **Client:** `addTrainClient.py` - Reads trains data from `trainList.txt` and sends it to `addTrainService.py`.

## 🎮 Simulation with RViz

In the second part of the project, RViz is used to visualize real-time 🚆 movement.

### 🎨 Marker Colors:

- 🔵 **Blue Cube:** Departure Station.
- 🔴 **Red Cube:** Arrival Station.
- 🟢 **Green Sphere:** On-time train.
- 🟡 **Yellow Sphere:** Delayed train.
- 🌸 **Pink Sphere:** Canceled train (remains at departure station).

### 🏃 Execution:

- 📍 `stationPublisher.py` - Publishes station markers.
- 🚆 `trainSimulator.py` - Manages trains movement and visualization.

### ⚙️ RViz Setup:

1️⃣ Open **RViz**.
2️⃣ Add topics:
   - ➕ `Add -> Topics -> /stations` (for 🏢 stations)
   - ➕ `Add -> Topics -> /trains` (for 🚆 trains)

## 🔧 Installation & Setup

1️⃣ Create a **ROS workspace**:
   ```sh
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   catkin_init_workspace
   ```
2️⃣ Clone the **repository**:
   ```sh
   git clone https://github.com/yourusername/train-simulation.git
   ```
3️⃣ Build the **workspace**:
   ```sh
   cd ~/catkin_ws
   catkin_make
   ```
4️⃣ Source the **workspace**:
   ```sh
   source devel/setup.bash
   ```
5️⃣ Run the **services & publishers**:
   ```sh
   rosrun train_simulation stationPublisher.py
   rosrun train_simulation trainSimulator.py
   ```
6️⃣ Open **RViz** and add topics as described above.

## 📜 License

This project is licensed under the **MIT License**. ✅

