def count_digits_before_decimal(number):
    number_str = str(number)
    if '.' in number_str:
        decimal_index = number_str.index('.')
        digits_before_decimal = decimal_index
    else:
        digits_before_decimal = len(number_str)
    return digits_before_decimal


Aloo = count_digits_before_decimal(1205)
print(Aloo)