import dearpygui.dearpygui as dpg
import launch

def main():
    dpg.create_context()

    with dpg.window(label="Tutorial", tag="Primary Window"):
        with dpg.table(header_row=False):

            # use add_table_column to add columns to the table,
            # table columns use child slot 0
            dpg.add_table_column()
            dpg.add_table_column()
            dpg.add_table_column()

            # add_table_next_column will jump to the next row
            # once it reaches the end of the columns
            # table next column use slot 1
            for i in range(0, 4):
                with dpg.table_row():
                    for j in range(0, 3):
                        dpg.add_text(f"Row{i} Column{j}")

    dpg.create_viewport(title='Custom Title', width=1000, height=500)
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.set_primary_window("Primary Window", True)
    dpg.start_dearpygui()
    dpg.destroy_context()
    
if __name__ == "__main__":
    main()
    