# test_color.py
import colorama
from colorama import Fore, Style

colorama.init(autoreset=True)

print(Fore.RED + "Red")
print(Fore.GREEN + "Green")
print(Fore.YELLOW + Style.BRIGHT + "Bright Yellow")
print(Style.RESET_ALL)
