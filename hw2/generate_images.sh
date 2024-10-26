#!/bin/bash

# Set the desired resolution and shading mode
xres=800   # Horizontal resolution
yres=800   # Vertical resolution

# Create the images directory if it doesn't exist
mkdir -p ./images
make clean && make

# Loop over each .txt scene file in the ./data/ directory
for scene_file in ./data/*.txt; do
    for mode in 0 1; do
        # Get the base filename without the directory and extension
        filename=$(basename -- "$scene_file")
        base="${filename%.*}"

        # Define the output image filenames
        # if mode = 0, then end name with 'Gouraud', else 'Phong'
        if [ $mode -eq 0 ]; then
            output_ppm="./images/${base}_Gouraud.ppm"
            output_png="./images/${base}_Gouraud.png"
        else
            output_ppm="./images/${base}_Phong.ppm"
            output_png="./images/${base}_Phong.png"
        fi

        # Run the rendering program and redirect the output to a .ppm file
        ./shaded "$scene_file" "$xres" "$yres" "$mode" > "$output_ppm"

        # Check if the rendering was successful
        if [ $? -eq 0 ]; then
            echo "Generated PPM image for $scene_file as $output_ppm"
        else
            echo "Error generating image for $scene_file"
            continue
        fi

        # Optionally, convert the .ppm file to .png format
        if command -v pnmtopng &> /dev/null; then
            pnmtopng "$output_ppm" > "$output_png"
            if [ $? -eq 0 ]; then
                echo "Converted $output_ppm to $output_png"
                # Uncomment the next line if you want to delete the .ppm files after conversion
                rm "$output_ppm"
            else
                echo "Error converting $output_ppm to PNG format"
            fi
        else
            echo "pnmtopng command not found. Skipping PNG conversion."
        fi
    done
done
