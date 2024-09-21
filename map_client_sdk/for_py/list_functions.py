# list_pymtmap_functions.py

def list_pymtmap_functions():
    try:
        import pymtmap
    except ImportError:
        print("Error: Unable to import pymtmap module.")
        return

    functions = dir(pymtmap)
    print("Functions and attributes in pymtmap module:")
    for func in functions:
        print(func)

if __name__ == "__main__":
    list_pymtmap_functions()
