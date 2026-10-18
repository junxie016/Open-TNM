# About the Algorithms

This code implements a bush-based algorithm that accurately solves maximum entropy user equilibrium (MEUE) traffic assignment problems without enumerating all user-equilibrium (UE) paths. For comparison, it also provides a method for finding a solution that satisfies the order-2 proportionality (O2P) condition; this method is based on the EMARB algorithm proposed by Xie and Nie (2019).

In this project, the `MEUE_TAPAS` class implements the MEUE algorithm, while the `TAP_iOEMA` class implements the O2P algorithm.

# How to Run the Algorithms

- Open `./MEUE/Source/TNA/TNA.sln` using Visual Studio 2019.
- Select `Release | x64` and set `TNADriver` as the startup project.
- Open `TNADriver.cpp` and modify `path` and `netname` in `main()` for the target network.
- Store the input files as `<netname>_net.tntp` and `<netname>_trips.tntp` under `path`.
- Use `Ctrl + F5` to compile and execute the code.

The bundled Chicago Sketch example uses:

```cpp
string path = "..\\..\\..\\Network\\cs\\";
string netname = "ChicagoSketch";
```

# Main Outputs

The two result files are written under `path`:

- `MEUE_olinkflow.txt`: origin-based link flows of the MEUE solution.
- `O2P_olinkflow.txt`: origin-based link flows of the O2P solution.

Both files are tab-delimited text files with the same format:

```text
origin  tail_node  head_node  flow
```

Each subsequent row gives the origin node, link tail node, link head node, and origin-based link flow. Rows with flow no greater than `1e-10` are omitted.


# Publications

Feng, L., Xie, J., Liu, X., Tang, Y., Wang, D. Z. W., and Nie, Y. M. (2024). Is order-2 proportionality good enough for approximating the most likely path flow in user equilibrium traffic assignment? *Transportation Research Part B: Methodological*, 186, 103007. 

Xie, J. and Nie, Y. M. (2019). A new algorithm for achieving proportionality in user equilibrium traffic assignment. *Transportation Science*, 53(2), 566-584.
