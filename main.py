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
    rules_file_path = "rules_table_2.xlsx"
    rules_df = load_rules_table(rules_file_path)
    if rules_df is not None:
        while True:
            situation_vector = []
            for column in rules_df.columns[1:]:  # Пропускаем первый столбец (Решения)
                col_name, value_options = column.split('(')
                value_options = value_options.strip(')').split('/')
                while True:
                    value = input(f"{col_name} ({'/'.join(value_options)}): ")
                    if value not in value_options:
                        print(f"Неверное значение. Пожалуйста, введите одно из следующих: {', '.join(value_options)}")
                        continue
                    else:
                        situation_vector.append('1' if value == '1' else '0' if value == '0' else value)
                        break

            print(situation_vector)
            decision = interpret_situation(situation_vector, rules_df)
            print("Решение:", decision)


if __name__ == "__main__":
    main()
