#!/bin/bash

# Loop 10 times
for i in {0..39}; do
  # Generate two random integers between 1 and 100 (inclusive)
  num1=3
  num2=$((i))

  # Run the Python script with the random numbers as arguments.
  # ***IMPORTANT: Quote the variables!***
  python3 main.py "$num1" "$num2"  

  # Check the exit status of the Python script.  This is VERY important
  # for error handling!
  if [[ $? -ne 0 ]]; then
    echo "Error: Python script failed on iteration $i"
    exit 1  # Exit the bash script if the Python script fails
  fi


  # Optional: Add a separator or some output to distinguish iterations
  echo "--- Iteration $i ---"
  sleep 5 # Optional: Pause for 1 second between iterations
done

echo "Finished iterations."