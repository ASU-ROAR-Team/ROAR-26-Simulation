import os
import numpy as np

def convert_npy_to_csv(input_dir, output_dir):
    """
    Finds all .npy files in the input directory and converts them to .csv in the output directory.
    """
    print(f"Scanning directory: {input_dir} for .npy files...\n")
    
    # Ensure the output directory exists, create it if it doesn't
    os.makedirs(output_dir, exist_ok=True)
    
    # Iterate through all files in the input directory
    for filename in os.listdir(input_dir):
        if filename.endswith(".npy"):
            npy_path = os.path.join(input_dir, filename)
            csv_path = os.path.join(output_dir, filename.replace(".npy", ".csv"))
            
            try:
                # Load the NumPy array
                data = np.load(npy_path)
                
                # Save as CSV. fmt='%f' ensures floats aren't written in scientific notation
                np.savetxt(csv_path, data, delimiter=",", fmt='%f')
                print(f"✅ Converted: {filename} -> {os.path.basename(csv_path)}")
                
            except Exception as e:
                print(f"❌ Failed to convert {filename}: {e}")

if __name__ == "__main__":
    # Get the absolute path of the directory where this script is located
    current_directory = os.path.dirname(os.path.abspath(__file__))
    
    # Define the inputs and outputs folder paths
    inputs_folder = os.path.join(current_directory, "inputs")
    outputs_folder = os.path.join(current_directory, "outputs")
    
    # Check if the inputs folder exists before running
    if not os.path.exists(inputs_folder):
        print(f"❌ The directory '{inputs_folder}' does not exist.")
        print("Please create an 'inputs' folder next to this script and put your .npy files inside.")
    else:
        convert_npy_to_csv(inputs_folder, outputs_folder)