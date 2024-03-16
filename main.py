import pandas as pd


def load_rules_table(file_path):
    try:
        rules_df = pd.read_excel(file_path)  # Предполагается, что файл в формате Excel
        return rules_df
    except FileNotFoundError:
        print("Файл не найден.")
        return None


def interpret_situation(situation_vector, rules_df):
    prev_decision = None
    for index, row in rules_df.iterrows():
        conditions = row[1:]  # Условия в строке (все, кроме первого столбца)
        decision = row.iloc[0]  # Решение (первый столбец)
        if pd.isna(decision):  # Если решение NaN
            decision = prev_decision  # Используем предыдущее значение решения
        else:
            prev_decision = decision  # Сохраняем текущее значение решения
        matched = True
        for i, condition in enumerate(conditions):
            if condition != '-' and str(condition) != str(situation_vector[i]):
                matched = False
                break
        if matched:
            return decision
    return "Решение не найдено"


def main():
    rules_file_path = "rules_table.xlsx"
    rules_df = load_rules_table(rules_file_path)
    if rules_df is not None:
        while True:
            situation_vector = input("Введите ситуационный вектор (0, 1 через пробел): ").split()
            if len(situation_vector) != len(rules_df.columns) - 1:
                print("Неверное количество элементов в векторе.")
                continue
            if not all(element in ['0', '1'] for element in situation_vector):
                print("Неверные значения в векторе. Пожалуйста, введите только 0 или 1.")
                continue
            situation_vector = ['Да' if value == '1' else 'Нет' for value in situation_vector]
            print(situation_vector)
            decision = interpret_situation(situation_vector, rules_df)
            print("Решение:", decision)


if __name__ == "__main__":
    main()
