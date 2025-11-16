#!/usr/bin/env python3
"""
STM32 Nucleo-L432KC: STM32 Pin to Arduino Label Mapper
"""

def get_pin_to_arduino():
    """Maps STM32 pin names to Arduino labels"""
    
    mapping = {
        # CN3 Left Side
        'PA9': 'D1',
        'PA10': 'D0',
        'PA12': 'D2',
        'PB0': 'D3',
        'PB7': 'D4',
        'PB6': 'D5',
        'PB1': 'D6',
        'PC14': 'D7',
        'PC15': 'D8',
        'PA8': 'D9',
        'PA11': 'D10',
        'PB5': 'D11',
        'PB4': 'D12',
        
        # CN4 Right Side
        'PA2': 'A7',
        'PA7': 'A6',
        'PA6': 'A5',
        'PA5': 'A4',
        'PA4': 'A3',
        'PA3': 'A2',
        'PA1': 'A1',
        'PA0': 'A0',
        'PB3': 'D13'
    }
    
    return mapping

def main():
    mapping = get_pin_to_arduino()
    
    print("=" * 40)
    print("Nucleo-L432KC Pin Mapper")
    print("=" * 40)
    print(f"{'STM32 Pin':^18} | {'Arduino Label':^18}")
    print("-" * 40)
    
    for stm_pin, arduino_label in sorted(mapping.items()):
        print(f"{stm_pin:^18} | {arduino_label:^18}")
    
    print("=" * 40)
    print("\nEnter STM32 pin (e.g., PA10, PB7) or 'quit'\n")
    
    while True:
        user_input = input("Enter pin: ").strip().upper()
        
        if user_input.lower() == 'quit':
            print("Goodbye!")
            break
        
        if not user_input:
            continue
        
        if user_input in mapping:
            print(f"  {user_input} → {mapping[user_input]}\n")
        else:
            print(f"  {user_input} not found\n")

if __name__ == "__main__":
    main()
