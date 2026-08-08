import os
import numpy as np

def convert_npy_to_csv(directory):
    """
    Finds all .npy files in the given directory and converts them to .csv
    """
    print(f"Scanning directory: {directory} for .npy files...\n")
    
    # Iterate through all files in the directory
    for filename in os.listdir(directory):
        if filename.endswith(".npy"):
            npy_path = os.path.join(directory, filename)
            csv_path = os.path.join(directory, filename.replace(".npy", ".csv"))
            
            try:
                # Load the NumPy array
                data = np.load(npy_path)
                
                # Save as CSV. fmt='%f' ensures floats aren't written in scientific notation
                # Adjust the delimiter if your optimal_path script expects spaces or tabs
                np.savetxt(csv_path, data, delimiter=",", fmt='%f')
                print(f"✅ Converted: {filename} -> {os.path.basename(csv_path)}")
                
            except Exception as e:
                print(f"❌ Failed to convert {filename}: {e}")

if __name__ == "__main__":
    # Get the absolute path of the directory where this script is located
    current_directory = os.path.dirname(os.path.abspath(__file__))
    convert_npy_to_csv(current_directory)
